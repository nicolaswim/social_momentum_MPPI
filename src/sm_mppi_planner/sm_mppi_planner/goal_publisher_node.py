#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.goal_sent = False

    def publish_goal(self):
        if self.goal_sent:
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.0
        goal.pose.position.y = 1.0
        goal.pose.orientation.w = 1.0
        self.publisher_.publish(goal)
        self.get_logger().info('Published goal pose.')
        self.goal_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin_once(node, timeout_sec=2.0)  # allow publishing
    node.destroy_node()
    rclpy.shutdown()
