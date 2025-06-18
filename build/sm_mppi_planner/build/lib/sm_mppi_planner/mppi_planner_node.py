#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration # Added for the settling period
from geometry_msgs.msg import Twist, PoseStamped
import torch
import numpy as np
import math
import time # time.time() is wall time, consider using self.get_clock().now() for ROS time operations
import sys
import yaml # <-- Make sure this import is at the top of your file

# --- Relative imports for modules within this package ---
try:
    from .tf2_wrapper import TF2Wrapper
    from .config import *
    from .utils import dynamics, normalize_angle
    from .sm_mppi import SMMPPIController
    from shapely.geometry import Polygon, MultiPolygon
    from shapely.vectorized import contains
except ImportError as e:
    print(f"Could not import local modules with relative paths: {e}", file=sys.stderr)
    print("Attempting direct imports (ensure PYTHONPATH is set if running outside ROS build system):", file=sys.stderr)
    from tf2_wrapper import TF2Wrapper
    from config import *
    from utils import dynamics, normalize_angle
    from sm_mppi import SMMPPIController
    from shapely.geometry import Polygon, MultiPolygon
    from shapely.vectorized import contains


class MPPLocalPlannerMPPI(Node):
    def __init__(self):
        super().__init__('mppi_planner_node')

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"MPPI Local Planner Node Initializing... Device: {self.device}")

        # --- ADDED: GOAL TOLERANCE PARAMETERS AND STATE ---
        self.declare_parameter('goal_xy_tolerance', 0.15)  # 15 cm
        self.declare_parameter('goal_yaw_tolerance', 0.15) # ~8.5 degrees
        self.goal_xy_tol = self.get_parameter('goal_xy_tolerance').value
        self.goal_yaw_tol = self.get_parameter('goal_yaw_tolerance').value

        self.goal = None # Will store the full goal [x, y, yaw]
        self.goal_reached = False # State flag

        
        # --- Logic to load obstacles from launch parameters ---
        self.declare_parameter('static_obstacles_yaml', STATIC_OBSTACLES_YAML_DEFAULT)
        static_obstacles_str = self.get_parameter('static_obstacles_yaml').value
        
        final_obstacles = []
        try:
            parsed_obstacles = yaml.safe_load(static_obstacles_str)
            if isinstance(parsed_obstacles, list) and len(parsed_obstacles) > 0:
                final_obstacles = parsed_obstacles
                self.get_logger().info(f"Successfully loaded {len(final_obstacles)} static obstacles from launch file.")
            else:
                self.get_logger().info("No valid static obstacles in launch file, using default from config.py.")
                final_obstacles = STATIC_OBSTACLES
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing static_obstacles_yaml: {e}. Using default from config.py.")
            final_obstacles = STATIC_OBSTACLES
        # --- End of obstacle loading logic ---

        # ----------------------------------------------------

        self.tf2_wrapper = TF2Wrapper(self)

        self.cmd_vel_pub = self.create_publisher(Twist, "/mobile_base_controller/cmd_vel_unstamped", 10)

        timer_period = 0.05  # seconds (for 20Hz)
        self.get_logger().info(f"Planner timer period set to: {timer_period} seconds (20 Hz)")
        self.timer = self.create_timer(timer_period, self.plan_and_publish)

        # self.start_time = time.time() # Wall time, consider ROS time if comparing with ROS timestamps

        self.controller = SMMPPIController(final_obstacles, self.device) # STATIC_OBSTACLES from config.py
        self.loop_counter = 0

        self.planner_fully_ready_time = None

        self.current_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)
        self.robot_velocity = torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device)
        self.previous_robot_state = torch.zeros_like(self.current_state)

        self.agent_states = {i: torch.tensor([100.0, 100.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.agent_velocities = {i: torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.previous_agent_states = {i: torch.zeros_like(self.agent_states[0]) for i in range(ACTIVE_AGENTS)}

        self.goal_topic_name = "/goal_pose"
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            self.goal_topic_name,
            self.goal_callback,
            10
        )
        self.get_logger().info(f"Subscribing to goal topic: {self.goal_topic_name}")

        # --- Additions for Initial Planner Settling Period ---
        self.planner_fully_ready_time = None
        # Wait 1.0 seconds after planner's own init before trusting human TFs fully.
        # This should be greater than or comparable to the fake_human_publisher's initial_delay_sec
        # plus a little extra for TF propagation.
        self.planner_initial_tf_wait_duration = Duration(seconds=1.0) # Adjusted from 0.5 to 1.0 for more safety
        # --- End Additions ---

        self.get_logger().info("MPPI Local Planner Node Initialized Successfully.")
        

    def goal_callback(self, msg: PoseStamped):
            if msg.header.frame_id != "map" and msg.header.frame_id != "odom":
                self.get_logger().warn(f"Received goal in frame '{msg.header.frame_id}', planner works in 'odom' frame. Ensure TF is available.")

            # --- MODIFIED TO STORE FULL GOAL AND RESET FLAG ---
            q = msg.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            
            new_goal_x = msg.pose.position.x
            new_goal_y = msg.pose.position.y

            self.get_logger().info(f"Received new external goal via topic: [{new_goal_x:.2f}, {new_goal_y:.2f}]")

            # Store the full goal for our tolerance check
            self.goal = torch.tensor([new_goal_x, new_goal_y, yaw], dtype=torch.float32).to(self.device)
            
            # Update the controller's goal (as you had before)
            self.controller.goal = torch.tensor([new_goal_x, new_goal_y], dtype=torch.float32).to(self.device)
            
            # Reset the controller's internal state (as you had before)
            self.controller.current_goal_index = -1
            self.controller.cycle_count = 0
            
            # IMPORTANT: Reset our state flag
            self.goal_reached = False
            # ---------------------------------------------------

    def plan_and_publish(self):

        # --- ADDED: STABLE GOAL CHECKING LOGIC AT THE TOP ---
        if self.goal is None:
            return # Don't do anything if no goal has been received

        # Calculate position and angle errors
        xy_error = torch.norm(self.current_state[:2] - self.goal[:2])
        yaw_error = abs(normalize_angle(self.current_state[2] - self.goal[2]))

        # Check if we are inside the tolerance zone
        if xy_error < self.goal_xy_tol and yaw_error < self.goal_yaw_tol:
            if not self.goal_reached:
                self.get_logger().info(f"Goal reached! XY Error: {xy_error:.3f}m, Yaw Error: {yaw_error:.3f}rad. Halting robot.")
                # Publish a single stop command
                self.cmd_vel_pub.publish(Twist())
                self.goal_reached = True # Set the flag so we don't spam logs
            
            # Exit the function early to do nothing else
            return
        
        # If we are here, we are not at the goal yet.
        self.goal_reached = False
        # --- END OF NEW GOAL CHECKING LOGIC ---
        # --- Initial Planner Settling Period Check ---
        if self.planner_fully_ready_time is None:
            # Set the time when the planner considers TFs to be settled, based on ROS time
            self.planner_fully_ready_time = self.get_clock().now() + self.planner_initial_tf_wait_duration

        current_ros_time = self.get_clock().now()
        if current_ros_time < self.planner_fully_ready_time:
            # Convert ROS Time to seconds for logging if needed
            planner_ready_time_sec = self.planner_fully_ready_time.nanoseconds / 1e9
            current_ros_time_sec = current_ros_time.nanoseconds / 1e9
            self.get_logger().info(
                f"Planner in initial TF settling period (current: {current_ros_time_sec:.2f}s, ready at: {planner_ready_time_sec:.2f}s). Publishing stop command.",
                throttle_duration_sec=self.planner_initial_tf_wait_duration.nanoseconds/1e9 + 0.5
            )
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return # Skip all planning and TF lookups during this period
        # --- End Settling Period Check ---

        self.loop_counter += 1

        # Assuming "odom" is your fixed frame for planning.
        # Robot pose: transform from "odom" (target) to "base_link" (source)
        T_odom_baselink_transform_stamped = self.tf2_wrapper.get_latest_pose("odom", "base_link")

        if T_odom_baselink_transform_stamped is None: # Note: your TF2Wrapper returns the .transform part
            self.get_logger().warn("Can't find robot pose (odom to base_link transform). Skipping planning cycle.", throttle_duration_sec=2.0)
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return

        # If TF2Wrapper returns the .transform attribute directly:
        robot_transform = T_odom_baselink_transform_stamped

        q_x = robot_transform.rotation.x
        q_y = robot_transform.rotation.y
        q_z = robot_transform.rotation.z
        q_w = robot_transform.rotation.w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        new_robot_state_list = [
            robot_transform.translation.x,
            robot_transform.translation.y,
            yaw,
        ]
        new_robot_state = torch.tensor(new_robot_state_list, dtype=torch.float32).to(self.device)

        # HZ from config.py is likely your control frequency (1/timer_period)
        # If HZ is 0.05, this is incorrect. HZ should be 1.0 / timer_period (e.g., 20.0)
        # Or, delta_time for velocity calculation should be self.timer.timer_period_ns / 1e9
        delta_time_for_velocity = 1.0 / (1.0/self.timer.timer_period_ns *1e9) if self.timer.timer_period_ns > 0 else 0.05 # Safer way to get dt
        if self.loop_counter > 1 and delta_time_for_velocity > 1e-9 : # Ensure delta_time is valid
            delta_pos = new_robot_state[:2] - self.previous_robot_state[:2]
            self.robot_velocity = delta_pos / delta_time_for_velocity
        else:
            self.robot_velocity = torch.tensor([0.0,0.0], dtype=torch.float32).to(self.device)


        self.current_state = new_robot_state
        self.previous_robot_state = self.current_state.clone()

        self.get_logger().info(f"Robot State (x,y,th): [{self.current_state[0]:.2f}, {self.current_state[1]:.2f}, {self.current_state[2]:.2f}] | Goal: {self.controller.goal.cpu().numpy()}", throttle_duration_sec=1.0)

        # Reset agent states to default (far away) before trying to update them
        # This ensures if a TF is missed, the agent isn't stuck at its last valid position indefinitely in the controller's view
        # (Though your current default is 100,100 which is already far)
        current_agent_states_for_controller = {
            i: self.agent_states[i].clone() for i in range(ACTIVE_AGENTS) # Start with last known or default
        }

        if ACTIVE_AGENTS > 0:
            for i in range(ACTIVE_AGENTS):
                human_frame_name = f"{HUMAN_FRAME}_{i}"
                # Human pose: transform from "odom" (target) to "human_X" (source)
                T_odom_human_transform = self.tf2_wrapper.get_latest_pose("odom", human_frame_name)

                if T_odom_human_transform is None:
                    self.get_logger().warn(f"Can't find human pose for {human_frame_name}. Using default/last known for this cycle.", throttle_duration_sec=1.0, throttle_time_source_type=self.get_clock())
                    # current_agent_states_for_controller[i] remains the default/last known
                else:
                    q_ax = T_odom_human_transform.rotation.x
                    q_ay = T_odom_human_transform.rotation.y
                    q_az = T_odom_human_transform.rotation.z
                    q_aw = T_odom_human_transform.rotation.w
                    agent_yaw = math.atan2(2 * (q_aw * q_az + q_ax * q_ay), 1 - 2 * (q_ay * q_ay + q_az * q_az))

                    new_agent_state_list = [T_odom_human_transform.translation.x, T_odom_human_transform.translation.y, agent_yaw]
                    new_agent_state = torch.tensor(new_agent_state_list, dtype=torch.float32).to(self.device)
                    new_agent_state[2] = normalize_angle(new_agent_state[2])

                    if self.loop_counter > 1 and delta_time_for_velocity > 1e-9: # and previous_agent_states exists
                        delta_agent_pos = new_agent_state[:2] - self.previous_agent_states[i][:2]
                        self.agent_velocities[i] = delta_agent_pos / delta_time_for_velocity
                    else:
                        self.agent_velocities[i] = torch.tensor([0.0,0.0], dtype=torch.float32).to(self.device)


                    current_agent_states_for_controller[i] = new_agent_state # Update with current valid pose
                    self.previous_agent_states[i] = new_agent_state.clone() # Store for next velocity calculation


        # Use the potentially updated current_agent_states_for_controller
        action, costs, rollouts, termination = self.controller.compute_control(
            self.current_state,
            self.previous_robot_state, # Should be self.current_state from *previous* iteration
            self.robot_velocity,
            current_agent_states_for_controller, # Pass the collected states for this cycle
            self.previous_agent_states, # This will be from the previous cycle where TFs were valid
            self.agent_velocities
        )

        if action is not None:
            self.get_logger().info(f"MPPI Action: [lin_vel={action[0]:.2f}, ang_vel={action[1]:.2f}], Termination: {termination}", throttle_duration_sec=0.5)
        else:
            self.get_logger().warn(f"MPPI Action is None. Termination: {termination}", throttle_duration_sec=1.0)

        if termination:
             self.get_logger().info(f"Termination reported by controller. Current pos: {self.current_state[:2].cpu().numpy()}, Goal: {self.controller.goal.cpu().numpy()}")

# --- ADD THIS NEW, SIMPLER BLOCK ---
        if action is not None:
            # If the planner gives a valid action, publish it
            twist_msg = Twist()
            twist_msg.linear.x = min(float(action[0].item()), VMAX)
            twist_msg.angular.z = float(action[1].item())
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f"Published cmd_vel: lin_x={twist_msg.linear.x:.2f}, ang_z={twist_msg.angular.z:.2f}", throttle_duration_sec=0.5)
        else:
            # If planner fails for any reason, stop the robot as a safety measure
            self.get_logger().warn("MPPI controller returned None action. Publishing zero velocity.", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(Twist())
        # --- END OF NEW BLOCK ---

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MPPLocalPlannerMPPI()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Ctrl+C detected, shutting down node.")
    except Exception as e:
        error_logger = rclpy.logging.get_logger('mppi_planner_node_main_exception') if not node else node.get_logger()
        error_logger.error(f"Exception in node execution: {e}")
        import traceback
        error_logger.error(traceback.format_exc())
    finally:
        if node and rclpy.ok():
            if hasattr(node, 'cmd_vel_pub') and node.cmd_vel_pub:
                node.get_logger().info("Publishing stop command before destroying node.")
                stop_cmd = Twist() # Ensure Twist is defined or imported if used here standalone
                node.cmd_vel_pub.publish(stop_cmd)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("MPPI Planner Node Shutdown complete.")

if __name__ == '__main__':
    main()