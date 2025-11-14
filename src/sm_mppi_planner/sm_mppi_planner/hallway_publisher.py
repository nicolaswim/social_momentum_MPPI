#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import math

class HallwayPublisher(Node):
    def __init__(self):
        super().__init__('hallway_publisher')
        
        # --- Declare parameters with default values ---
        self.declare_parameter('hallway_width', 5.0)
        self.declare_parameter('hallway_length', 20.0)
        self.declare_parameter('wall_height', 2.5)
        self.declare_parameter('wall_thickness', 0.2)
        self.declare_parameter('wall_mesh_path', 'package://sm_mppi_planner/models/wall/simple_wall.dae')

        # --- Get the parameters from the launch file ---
        self.hallway_width = self.get_parameter('hallway_width').value
        self.hallway_length = self.get_parameter('hallway_length').value
        self.wall_height = self.get_parameter('wall_height').value
        self.wall_thickness = self.get_parameter('wall_thickness').value
        self.mesh_path = self.get_parameter('wall_mesh_path').value

        # Publisher for the MarkerArray
        self.publisher_ = self.create_publisher(MarkerArray, '/hallway_markers', 10)
        
        # Timer to publish periodically
        self.timer = self.create_timer(1.0, self.publish_hallway)
        
        self.get_logger().info("Configurable Hallway Publisher has started.")
        self.get_logger().info(f"Hallway dimensions (LxW): {self.hallway_length}m x {self.hallway_width}m")


    def publish_hallway(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # --- Wall 1 ---
        wall_1 = Marker()
        wall_1.header.frame_id = "odom"
        wall_1.header.stamp = now
        wall_1.ns = "hallway"
        wall_1.id = 0
        wall_1.type = Marker.MESH_RESOURCE
        wall_1.action = Marker.ADD
        wall_1.mesh_resource = self.mesh_path
        
        # Set pose from parameters
        wall_1.pose.position.x = 0.0
        wall_1.pose.position.y = self.hallway_width / 2.0
        wall_1.pose.position.z = 0.0
        wall_1.pose.orientation.w = 1.0

        # Set scale from parameters
        wall_1.scale.x = self.hallway_length
        wall_1.scale.y = self.wall_thickness
        wall_1.scale.z = self.wall_height

        # Set color
        wall_1.color.r = 0.7
        wall_1.color.g = 0.7
        wall_1.color.b = 0.7
        wall_1.color.a = 1.0

        # --- Wall 2 ---
        wall_2 = Marker()
        wall_2.header.frame_id = "odom"
        wall_2.header.stamp = now
        wall_2.ns = "hallway"
        wall_2.id = 1
        wall_2.type = Marker.MESH_RESOURCE
        wall_2.action = Marker.ADD
        wall_2.mesh_resource = self.mesh_path
        
        # Set pose from parameters
        wall_2.pose.position.x = 0.0
        wall_2.pose.position.y = -self.hallway_width / 2.0
        wall_2.pose.orientation.w = 1.0

        # Use the same scale and color
        wall_2.scale = wall_1.scale
        wall_2.color = wall_1.color

        # --- START: ADDED RED DOT MARKER ---
        
        goal_marker = Marker()
        goal_marker.header.frame_id = "odom"
        goal_marker.header.stamp = now
        goal_marker.ns = "goal_marker"
        goal_marker.id = 2  # Give it a unique ID
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        
        # Set pose to (9.0, 0.0, 0.0)
        goal_marker.pose.position.x = 9.0
        goal_marker.pose.position.y = 0.0
        goal_marker.pose.position.z = 0.0  # At ground level
        goal_marker.pose.orientation.w = 1.0

        # Set scale (e.g., 0.3m diameter)
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.3

        # Set color to red
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        
        # --- END: ADDED RED DOT MARKER ---

        marker_array.markers.append(wall_1)
        marker_array.markers.append(wall_2)
        marker_array.markers.append(goal_marker) # Add the dot to the array
        
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = HallwayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()