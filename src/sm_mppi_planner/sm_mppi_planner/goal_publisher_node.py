#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class GoalPublisherNode(Node):
    """
    A node that continuously publishes a static goal pose to the /goal_pose topic.
    The goal coordinates can be set via ROS 2 parameters.
    """
    def __init__(self):
        super().__init__('goal_publisher_node')
        
        # --- FIX: Declare parameters so they can be set from the launch file ---
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        
        # Get the goal coordinates from the parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        
        # Create the publisher
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Create the goal message once
        self.goal_msg = PoseStamped()
        self.goal_msg.header = Header()
        self.goal_msg.header.frame_id = 'map' 
        self.goal_msg.pose.position.x = self.goal_x
        self.goal_msg.pose.position.y = self.goal_y
        self.goal_msg.pose.position.z = 0.0
        self.goal_msg.pose.orientation.w = 1.0

        # Create a timer to publish the goal periodically (e.g., once a second)
        # This makes it robust and ensures the planner always gets the goal.
        self.timer = self.create_timer(1.0, self.publish_goal)
        
        self.get_logger().info(f"Goal Publisher has started. Continuously publishing goal: ({self.goal_x}, {self.goal_y})")

    def publish_goal(self):
        """
        This method is called by the timer and publishes the goal message.
        """
        self.goal_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.goal_msg)

def main(args=None):
    rclpy.init(args=args)
    goal_publisher_node = GoalPublisherNode()
    try:
        rclpy.spin(goal_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        goal_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()