#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import torch

try:
    from .config import GOALS
except ImportError:
    from config import GOALS

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.declare_parameter('use_sim_time', True)
        self.get_logger().info("Goal Publisher Node started. Publishing goal from config.py...")
        self.publish_goal()

    def publish_goal(self):
        if not GOALS:
            self.get_logger().warn("No goals defined in config.py (GOALS is empty).")
            return

        goal = GOALS[0]  # Just take the first goal
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0  # Neutral orientation

        self.publisher.publish(msg)
        self.get_logger().info(f"Published goal to /goal_pose: x={goal[0]}, y={goal[1]}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    rclpy.spin_once(node, timeout_sec=2.0)  # Publish once then shut down
    node.destroy_node()
    rclpy.shutdown()
