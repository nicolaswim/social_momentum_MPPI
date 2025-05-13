#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # MODIFIED: Changed from TwistStamped
# from tf2_wrapper import TF2Wrapper # Assuming this will be in the same directory or Python path
# from pytorch_mppi import MPPI # Assuming this library is installed
# from config import * # Assuming this will be in the same directory or Python path
# from utils import dynamics, normalize_angle, save_data # Assuming this will be in the same directory or Python path
# from sm_mppi import SMMPPIController # Assuming this will be in the same directory or Python path
import torch # Assuming torch is installed
import numpy as np
import math
import time
# from shapely.geometry import Polygon, MultiPolygon, Point # Assuming shapely is installed
# from shapely.vectorized import contains # Assuming shapely is installed

# --- To make relative imports work correctly when this is part of a package ---
# It's good practice to ensure your custom modules are clearly imported.
# If they are in the same directory as this file (within the sm_mppi_planner Python package),
# Python's default import mechanism should find them.
# For clarity and robustness within a ROS 2 package structure,
# you might use explicit relative imports if these files are treated as part of the same module:
try:
    from .tf2_wrapper import TF2Wrapper
    from .config import * 
    from .utils import dynamics, normalize_angle, save_data
    from .sm_mppi import SMMPPIController
    from .vis_utils import VisualizationUtils # Make sure this is here if used
    from shapely.geometry import Polygon, MultiPolygon, Point 
    from shapely.vectorized import contains 
except ImportError as e:
    # This 'except' block attempting direct imports for your local modules is problematic
    # because those direct imports won't work when run as part of a package.
    # It's better to fix all relative imports so you don't hit this 'except' block for local modules.
    print(f"Could not import local modules with relative paths: {e}")
    print("Attempting direct imports (ensure PYTHONPATH is set if running outside ROS build system. This may fail for packaged modules.):")
    # Re-attempting direct imports here for local modules will likely still fail.
    # For now, let's leave it to see if fixing utils.py resolves the primary chain.
    # Ideally, this except block should only handle truly missing external libraries if any.
    from tf2_wrapper import TF2Wrapper # This will fail if not fixed relatively above
    from config import *
    from utils import dynamics, normalize_angle, save_data
    from sm_mppi import SMMPPIController
    from vis_utils import VisualizationUtils # This will fail if not fixed relatively above
    from shapely.geometry import Polygon, MultiPolygon, Point
    from shapely.vectorized import contains


class MPPLocalPlannerMPPI(Node):
    def __init__(self):
        super().__init__('mppi_planner_node') # MODIFIED: Renamed node for clarity, matches entry point
        self.get_logger().info(f"MPPI Local Planner Node Initializing... Device: {self.device}")


        # Initialize parameters (Ideally, use self.declare_parameter and self.get_parameter)
        # For now, it relies on importing from config.py

        self.tf2_wrapper = TF2Wrapper(self)
        # self.rollouts = torch.zeros((7, NUM_SAMPLES, 2)) # These seem unused in this class
        # self.costs = torch.zeros((7, NUM_SAMPLES, 2))   # These seem unused in this class
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # ROS2 setup
        # MODIFIED: Topic name and message type for TIAGo
        self.cmd_vel_pub = self.create_publisher(Twist, "/mobile_base_controller/cmd_vel_unstamped", 1)
        
        # HZ from config.py is 0.05, which means a timer period of 1/0.05 = 20 seconds.
        # This seems very slow for a planner. Assuming HZ in config.py meant "seconds per cycle"
        # If HZ is meant to be "frequency in Hz", then timer_period = 1.0 / HZ.
        # The original code had self.timer = self.create_timer(0.05, self.plan_and_publish)
        # Let's assume 0.05 was the intended timer_period (20 Hz)
        timer_period = 0.05 # seconds (for 20Hz) 
        # If HZ in config.py (0.05) was meant to be the period, then use that:
        # timer_period = HZ 
        self.get_logger().info(f"Planner timer period set to: {timer_period} seconds")
        self.timer = self.create_timer(timer_period, self.plan_and_publish)
        
        self.time_steps = []
        self.linear_velocities = []
        self.angular_velocities = []
        self.start_time = time.time()
        
        # Ensure STATIC_OBSTACLES from config.py is correctly formatted for SMMPPIController
        self.controller = SMMPPIController(STATIC_OBSTACLES, self.device)
        self.counter = 0 # For self.previous_robot_state logic

        # Initial states (ensure these match the expected tensor shapes and device)
        self.current_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)  # [x, y, yaw]
        self.robot_velocity = torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device) # [vx, vy] in world frame
        self.previous_robot_state = torch.zeros_like(self.current_state) # Initialize to zeros

        # Initialize agent states and velocities (using ACTIVE_AGENTS from config.py)
        self.agent_states = {i: torch.tensor([10.0, 10.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.agent_velocities = {i: torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.previous_agent_states = {i: torch.zeros_like(self.agent_states[0]) for i in range(ACTIVE_AGENTS)} # Initialize to zeros

        self.get_logger().info("MPPI Local Planner Node Initialized Successfully.")

    def plan_and_publish(self):
        timer_callback_start_time = time.time()
        self.counter += 1 # Increment counter

        T_map_baselink_transform = self.tf2_wrapper.get_latest_pose("map", "base_link") # Renamed for clarity
        
        if T_map_baselink_transform is None:
            self.get_logger().warn("Can't find robot pose (map to base_link transform). Skipping planning cycle.")
            # Optionally publish zero velocity to stop the robot if pose is lost
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        # Convert pose to MPPI state representation [x, y, yaw]
        # Your original yaw calculation from quaternion:
        # yaw = 2.0 * math.atan2(T_map_baselink_transform.rotation.z, T_map_baselink_transform.rotation.w)
        # This is a common way to get yaw if roll and pitch are zero.
        # Let's ensure the quaternion is normalized if needed, or use a more robust method if available.
        # For simplicity, we'll keep your method but be aware of potential gimbal lock if roll/pitch are large.
        q_x = T_map_baselink_transform.rotation.x
        q_y = T_map_baselink_transform.rotation.y
        q_z = T_map_baselink_transform.rotation.z
        q_w = T_map_baselink_transform.rotation.w

        # Standard formula for yaw from quaternion
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        new_robot_state = torch.tensor(
            [
                T_map_baselink_transform.translation.x,
                T_map_baselink_transform.translation.y,
                yaw,
            ],
            dtype=torch.float32,
        ).to(self.device)

        # Velocity calculation (world frame, [vx, vy])
        # Need a valid previous_robot_state to calculate velocity
        if self.counter > 1 : # Ensure previous_robot_state is not the initial zero/placeholder
            # HZ is 0.05 in config, meaning dt_for_vel_calc = 0.05 seconds.
            # If HZ was meant to be frequency, then dt_for_vel_calc = 1.0/HZ.
            # Assuming HZ from config is the delta_time for velocity calculation.
            # If the timer_period for plan_and_publish is different from HZ, this velocity might be less accurate.
            # For better accuracy, use actual time delta.
            dt_for_vel_calc = HZ # From your config.py
            # Or, more robustly, use the actual time between calls if HZ is not guaranteed.
            # For now, using HZ as per your original code.
            
            delta_pos = new_robot_state[:2] - self.previous_robot_state[:2]
            self.robot_velocity = delta_pos / dt_for_vel_calc
        
        self.current_state = new_robot_state # Update current_state AFTER using it for velocity calc if needed
        self.previous_robot_state = self.current_state.clone() # Update previous state for next iteration

        # Agent states and velocities
        for i in range(ACTIVE_AGENTS): # ACTIVE_AGENTS from config.py
            human_frame_name = HUMAN_FRAME + "_" + str(i + 1) # HUMAN_FRAME from config.py
            T_map_agent_transform = self.tf2_wrapper.get_latest_pose("map", human_frame_name)
            
            if T_map_agent_transform is None:
                self.get_logger().warn(f"Can't find human pose for {human_frame_name}. Using default/last known.")
                # Keep last known state or default if never seen.
                # agent_state = self.agent_states[i] # Keep last known state
            else:
                q_ax = T_map_agent_transform.rotation.x
                q_ay = T_map_agent_transform.rotation.y
                q_az = T_map_agent_transform.rotation.z
                q_aw = T_map_agent_transform.rotation.w
                agent_yaw = math.atan2(2 * (q_aw * q_az + q_ax * q_ay), 1 - 2 * (q_ay * q_ay + q_az * q_az))
                
                new_agent_state = torch.tensor(
                    [T_map_agent_transform.translation.x, T_map_agent_transform.translation.y, agent_yaw],
                    dtype=torch.float32
                ).to(self.device)
                new_agent_state[2] = normalize_angle(new_agent_state[2]) # normalize_angle from utils.py

                # Calculate agent velocity (world frame, [vx, vy])
                if self.counter > 1 : # Ensure previous_agent_states[i] is valid
                    delta_agent_pos = new_agent_state[:2] - self.previous_agent_states[i][:2]
                    self.agent_velocities[i] = delta_agent_pos / HZ # Using HZ as dt
                
                self.agent_states[i] = new_agent_state
                self.previous_agent_states[i] = self.agent_states[i].clone()


        # Compute optimal control using MPPI controller
        # Ensure the SMMPPIController.compute_control method expects these arguments in this order
        # and that their meaning (e.g., world frame vs base frame for velocities) matches.
        # Your SMMPPIController.compute_control takes:
        # (current_state, previous_robot_state_for_sm, robot_velocity_for_sm,
        #  agent_states_for_sm, previous_agent_states_for_sm, agent_velocities_for_sm)
        # The SMMPPIController's SocialCost uses self.robot_velocity and self.agent_velocities[i]
        # which are set in the SMMPPIController's compute_control from its arguments.
        
        # Here, self.robot_velocity and self.agent_velocities are already calculated in world frame.
        # The SMMPPIController's SocialCost expects velocities.
        action, costs, rollouts, termination = self.controller.compute_control(
            self.current_state, # Current robot state [x,y,theta] world
            self.previous_robot_state, # Used by SMMPPI for its own internal logic if needed
            self.robot_velocity, # Robot's current velocity [vx,vy] world
            self.agent_states,   # Dict of agent states [x,y,theta] world
            self.previous_agent_states, # Used by SMMPPI for its own internal logic if needed
            self.agent_velocities # Dict of agent velocities [vx,vy] world
        )
        # Storing rollouts and costs if needed for visualization
        # self.rollouts = rollouts 
        # self.costs = costs

        if action is not None and not termination:
            twist_msg = Twist()
            # VMAX is from your config.py, ensuring we don't exceed it
            twist_msg.linear.x = min(float(action[0].item()), VMAX) 
            twist_msg.angular.z = float(action[1].item())
            self.cmd_vel_pub.publish(twist_msg)
            
            # For data logging if needed
            self.linear_velocities.append(action[0].item())
            self.angular_velocities.append(action[1].item())
            self.time_steps.append(time.time() - self.start_time)
        elif termination:
            self.get_logger().info("Goal reached according to MPPI controller!")
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.controller.move_to_next_goal() # Logic from SMMPPIController
            # Potentially save data:
            # if hasattr(self.controller, 'save_plot'): # If save_plot is a method of SMMPPIController
            #     self.controller.save_plot() 
            # elif 'save_data' in globals(): # If save_data is the global function from utils
            #     # This needs to be adapted based on how save_data is intended to be called
            #     pass
        else:
            self.get_logger().warn("MPPI controller failed to compute optimal controls. Publishing zero velocity.")
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
        # timer_callback_processing_time = time.time() - timer_callback_start_time
        # self.get_logger().debug(f"Planner callback processing time: {timer_callback_processing_time:.4f}s")

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
        if node:
            node.get_logger().error(f"Exception in node: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
        else:
            print(f"Exception before node initialization: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    finally:
        if node: # Ensure node was successfully initialized
            if hasattr(node, 'cmd_vel_pub') and node.cmd_vel_pub:
                # Send a final stop command upon shutting down
                stop_cmd = Twist()
                node.cmd_vel_pub.publish(stop_cmd)
                node.get_logger().info("Published stop command before destroying node.")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("MPPI Planner Node Shutdown complete.")

if __name__ == '__main__':
    main()