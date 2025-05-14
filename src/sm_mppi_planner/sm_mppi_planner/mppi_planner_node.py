#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped # <--- ENSURED PoseStamped is imported
import torch
import numpy as np
import math
import time
import sys

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

        self.tf2_wrapper = TF2Wrapper(self)
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/mobile_base_controller/cmd_vel_unstamped", 10)
        
        timer_period = 0.05  # seconds (for 20Hz)
        self.get_logger().info(f"Planner timer period set to: {timer_period} seconds (20 Hz)")
        self.timer = self.create_timer(timer_period, self.plan_and_publish)
        
        self.start_time = time.time()
        
        self.controller = SMMPPIController(STATIC_OBSTACLES, self.device) # STATIC_OBSTACLES from config.py
        self.loop_counter = 0 

        self.current_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)
        self.robot_velocity = torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device)
        self.previous_robot_state = torch.zeros_like(self.current_state)

        self.agent_states = {i: torch.tensor([100.0, 100.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.agent_velocities = {i: torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.previous_agent_states = {i: torch.zeros_like(self.agent_states[0]) for i in range(ACTIVE_AGENTS)}

        self.goal_topic_name = "/goal_pose" 
        self.goal_subscriber = self.create_subscription(
            PoseStamped, # Now defined
            self.goal_topic_name,
            self.goal_callback,
            10 
        )
        self.get_logger().info(f"Subscribing to goal topic: {self.goal_topic_name}")

        self.get_logger().info("MPPI Local Planner Node Initialized Successfully.")

    def goal_callback(self, msg: PoseStamped): # Type hint PoseStamped is now valid
        if msg.header.frame_id != "map": 
            self.get_logger().warn(f"Received goal in frame '{msg.header.frame_id}', but planner operates in 'map'. TF transform would be needed.")
            # For now, assume goal is in 'map' frame.
            # If not, you would use self.tf2_wrapper to transform it here.

        new_goal_x = msg.pose.position.x
        new_goal_y = msg.pose.position.y
        
        self.get_logger().info(f"Received new external goal via topic: [{new_goal_x:.2f}, {new_goal_y:.2f}]")
        
        self.controller.goal = torch.tensor([new_goal_x, new_goal_y], dtype=torch.float32).to(self.device)
        
        self.controller.current_goal_index = -1 
        self.controller.cycle_count = 0 

    def plan_and_publish(self):
        self.loop_counter += 1

        T_map_baselink_transform = self.tf2_wrapper.get_latest_pose("odom", "base_link")
        
        if T_map_baselink_transform is None:
            self.get_logger().warn("Can't find robot pose (map to base_link transform). Skipping planning cycle.", throttle_duration_sec=2.0)
            stop_cmd = Twist() 
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        q_x = T_map_baselink_transform.rotation.x
        q_y = T_map_baselink_transform.rotation.y
        q_z = T_map_baselink_transform.rotation.z
        q_w = T_map_baselink_transform.rotation.w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        new_robot_state_list = [
            T_map_baselink_transform.translation.x,
            T_map_baselink_transform.translation.y,
            yaw,
        ]
        new_robot_state = torch.tensor(new_robot_state_list, dtype=torch.float32).to(self.device)

        if self.loop_counter > 1 : 
            delta_pos = new_robot_state[:2] - self.previous_robot_state[:2]
            self.robot_velocity = delta_pos / HZ # HZ from config.py
        
        self.current_state = new_robot_state 
        self.previous_robot_state = self.current_state.clone()

        self.get_logger().info(f"Robot State (x,y,th): [{self.current_state[0]:.2f}, {self.current_state[1]:.2f}, {self.current_state[2]:.2f}] | Goal: {self.controller.goal.cpu().numpy()}", throttle_duration_sec=1.0)

        if ACTIVE_AGENTS > 0:
            for i in range(ACTIVE_AGENTS):
                human_frame_name = HUMAN_FRAME + "_" + str(i + 1)
                T_map_agent_transform = self.tf2_wrapper.get_latest_pose("map", human_frame_name)
                
                if T_map_agent_transform is None:
                    self.get_logger().warn(f"Can't find human pose for {human_frame_name}. Using default/last known.", throttle_duration_sec=2.0)
                else:
                    q_ax = T_map_agent_transform.rotation.x
                    q_ay = T_map_agent_transform.rotation.y
                    q_az = T_map_agent_transform.rotation.z
                    q_aw = T_map_agent_transform.rotation.w
                    agent_yaw = math.atan2(2 * (q_aw * q_az + q_ax * q_ay), 1 - 2 * (q_ay * q_ay + q_az * q_az))
                    
                    new_agent_state_list = [T_map_agent_transform.translation.x, T_map_agent_transform.translation.y, agent_yaw]
                    new_agent_state = torch.tensor(new_agent_state_list, dtype=torch.float32).to(self.device)
                    new_agent_state[2] = normalize_angle(new_agent_state[2])

                    if self.loop_counter > 1:
                        delta_agent_pos = new_agent_state[:2] - self.previous_agent_states[i][:2]
                        self.agent_velocities[i] = delta_agent_pos / HZ
                    
                    self.agent_states[i] = new_agent_state
                    self.previous_agent_states[i] = self.agent_states[i].clone()
        
        action, costs, rollouts, termination = self.controller.compute_control(
            self.current_state,
            self.previous_robot_state, 
            self.robot_velocity,       
            self.agent_states,         
            self.previous_agent_states,
            self.agent_velocities      
        )
        
        if action is not None:
            self.get_logger().info(f"MPPI Action: [lin_vel={action[0]:.2f}, ang_vel={action[1]:.2f}], Termination: {termination}", throttle_duration_sec=0.5)
        else:
            self.get_logger().warn(f"MPPI Action is None. Termination: {termination}", throttle_duration_sec=1.0)

        if termination:
             self.get_logger().info(f"Termination reported by controller. Current pos: {self.current_state[:2].cpu().numpy()}, Goal: {self.controller.goal.cpu().numpy()}")

        if action is not None and not termination:
            twist_msg = Twist()
            twist_msg.linear.x = min(float(action[0].item()), VMAX) # VMAX from config.py
            twist_msg.angular.z = float(action[1].item())
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f"Published cmd_vel: lin_x={twist_msg.linear.x:.2f}, ang_z={twist_msg.angular.z:.2f}", throttle_duration_sec=0.5)
        elif termination:
            self.get_logger().info("Goal reached according to MPPI controller! Publishing zero velocity.")
            stop_cmd = Twist() 
            self.cmd_vel_pub.publish(stop_cmd)
            self.controller.move_to_next_goal() 
        else: 
            self.get_logger().warn("MPPI controller failed to compute optimal controls or action was None. Publishing zero velocity.")
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
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
                stop_cmd = Twist()
                node.cmd_vel_pub.publish(stop_cmd)
                node.get_logger().info("Published stop command before destroying node.")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("MPPI Planner Node Shutdown complete.")

if __name__ == '__main__':
    main()
