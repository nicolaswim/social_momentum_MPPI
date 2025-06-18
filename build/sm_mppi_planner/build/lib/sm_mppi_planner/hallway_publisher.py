#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import math

class HallwayPublisher(Node):
    def __init__(self):
        super().__init__('hallway_publisher')
        
        # Publisher for the MarkerArray
        self.publisher_ = self.create_publisher(MarkerArray, '/hallway_markers', 10)
        
        # Timer to publish periodically, ensuring the markers persist in RViz
        self.timer = self.create_timer(1.0, self.publish_hallway)
        
        self.get_logger().info("Hallway Publisher has started. Publishing walls on '/hallway_markers'.")

    def publish_hallway(self):
        # Create a MarkerArray to hold both of our walls
        marker_array = MarkerArray()

        # --- Hallway Configuration ---
        # IMPORTANT: Update this path to match where your .dae file is installed
        mesh_path = "package://sm_mppi_planner/models/wall/wall.dae"

        # Define the dimensions of your hallway
        hallway_width = 5.0  # Distance between the two walls (in meters)
        
        # NOTE ON SCALE:
        # The .dae file has its own size. The scale values multiply that size.
        # If your wall.dae is 1m long, a scale.x of 20.0 will make it 20m long.
        wall_length = 20.0
        wall_height = 2.5
        wall_thickness = 0.2

        # --- Wall 1 ---
        wall_1 = Marker()
        wall_1.header.frame_id = "odom"  # Your world frame
        wall_1.header.stamp = self.get_clock().now().to_msg()
        wall_1.ns = "hallway"
        wall_1.id = 0  # Unique ID for this marker
        wall_1.type = Marker.MESH_RESOURCE
        wall_1.action = Marker.ADD

        # Set the mesh resource to your wall file
        wall_1.mesh_resource = mesh_path
        
        # Set the pose of the first wall
        wall_1.pose.position.x = 0.0
        wall_1.pose.position.y = hallway_width / 2.0  # Position on the +Y side
        wall_1.pose.position.z = 0.0
        wall_1.pose.orientation.w = 1.0 # No rotation

        # Set the scale of the mesh
        wall_1.scale.x = wall_length
        wall_1.scale.y = wall_thickness
        wall_1.scale.z = wall_height

        # Set the color and transparency (r,g,b,a)
        wall_1.color.r = 0.7
        wall_1.color.g = 0.7
        wall_1.color.b = 0.7
        wall_1.color.a = 1.0

        # --- Wall 2 ---
        wall_2 = Marker()
        wall_2.header.frame_id = "odom" # Your world frame
        wall_2.header.stamp = self.get_clock().now().to_msg()
        wall_2.ns = "hallway"
        wall_2.id = 1 # Unique ID for this marker
        wall_2.type = Marker.MESH_RESOURCE
        wall_2.action = Marker.ADD
        wall_2.mesh_resource = mesh_path
        
        # Set the pose of the second wall
        wall_2.pose.position.x = 0.0
        wall_2.pose.position.y = -hallway_width / 2.0 # Position on the -Y side
        wall_2.pose.position.z = 0.0
        wall_2.pose.orientation.w = 1.0 # No rotation

        # Set the same scale and color
        wall_2.scale = wall_1.scale
        wall_2.color = wall_1.color

        # Add both walls to the MarkerArray
        marker_array.markers.append(wall_1)
        marker_array.markers.append(wall_2)

        # Publish the array
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()