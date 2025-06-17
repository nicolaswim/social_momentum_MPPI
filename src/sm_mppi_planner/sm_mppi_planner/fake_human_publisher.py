#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import random
import math
import sys
import os

# --- All necessary ROS Imports ---
from geometry_msgs.msg import Point, Pose, Twist, Quaternion, Vector3, TransformStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

try:
    from my_social_nav_interfaces.msg import HumanPoseVel, HumanArray
except ImportError:
    print("ERROR: Cannot import custom messages from my_social_nav_interfaces.", file=sys.stderr)
    print("Ensure the interface package is built and the current workspace is sourced.", file=sys.stderr)
    sys.exit(1)


class FakeHumanPublisher(Node):
    def __init__(self):
        super().__init__('fake_human_publisher')
        
        # --- Parameters ---
        self.declare_parameter('num_random_humans', 5)
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('human_max_speed', 1.5)
        self.declare_parameter('human_min_speed', 0.5)
        self.declare_parameter('world_frame_id', 'odom')
        self.declare_parameter('humans_topic', '/humans')
        self.declare_parameter('markers_topic', '/human_markers')
        self.declare_parameter('x_limits', [-7.0, 7.0])
        self.declare_parameter('y_limits', [-7.0, 7.0])
        self.declare_parameter('human_mesh_path', "")

        self.num_random = self.get_parameter('num_random_humans').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.max_speed = self.get_parameter('human_max_speed').value
        self.min_speed = self.get_parameter('human_min_speed').value
        self.frame_id = self.get_parameter('world_frame_id').value
        humans_topic = self.get_parameter('humans_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value
        self.mesh_path = self.get_parameter('human_mesh_path').value

        # --- Publishers and TF Broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.humans_publisher = self.create_publisher(HumanArray, humans_topic, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, markers_topic, 10)

        # --- Internal State ---
        self.humans = []
        
        # Initialize human data
        self._initialize_humans()

        # --- Main Timer ---
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.update_and_publish)
        self.get_logger().info("Fake Human Publisher has started.")

    def _initialize_humans(self):
        for i in range(self.num_random):
            human = {
                'id': f'human_{i}',
                'pos': np.array([random.uniform(self.x_lim[0], self.x_lim[1]),
                                 random.uniform(self.y_lim[0], self.y_lim[1])]),
                'vel': self.get_random_velocity()
            }
            self.humans.append(human)

    def get_random_velocity(self):
        speed = random.uniform(self.min_speed, self.max_speed)
        angle = random.uniform(0, 2 * math.pi)
        return np.array([speed * math.cos(angle), speed * math.sin(angle)])

    def update_and_publish(self):
        human_msgs = []
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # We need to send a "delete all" marker first to clear old markers
        # This is important for a variable number of objects
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        for i, human in enumerate(self.humans):
            human['pos'] += human['vel'] * self.timer_period
            
            # Bounce in bounds logic
            if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]) or \
               not (self.y_lim[0] < human['pos'][1] < self.y_lim[1]):
                human['vel'] *= -1.0 

            # --- Publish TF for the planner (this is still needed) ---
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = human['id']
            t.transform.translation = Vector3(x=human['pos'][0], y=human['pos'][1], z=0.0)
            yaw = math.atan2(human['vel'][1], human['vel'][0])
            q = Quaternion(z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

            # --- Build the HumanArray message (for other nodes) ---
            hpv_msg = HumanPoseVel()
            hpv_msg.human_id = human['id']
            hpv_msg.position = Point(x=human['pos'][0], y=human['pos'][1], z=0.0)
            hpv_msg.velocity = Vector3(x=human['vel'][0], y=human['vel'][1], z=0.0)
            human_msgs.append(hpv_msg)

            # --- Create the MESH_RESOURCE Marker for RViz ---
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "human_meshes"
            marker.id = i # Use the index as the ID
            marker.action = Marker.ADD

            # Set the type to MESH_RESOURCE
            marker.type = Marker.MESH_RESOURCE
            # Point to the 3D model file
            marker.mesh_resource = self.mesh_path
            marker.mesh_use_embedded_materials = True
            
            # Set the pose of the human model
            marker.pose.position.x = human['pos'][0]
            marker.pose.position.y = human['pos'][1]
            marker.pose.position.z = 0.6 # Adjust z so feet are on the ground
            marker.pose.orientation = q
            
            # Set the scale
            marker.scale = Vector3(x=1., y=1., z=1.)

            # Add the marker to our array
            marker_array.markers.append(marker)

        # Publish the topics
        self.humans_publisher.publish(HumanArray(header=Header(stamp=now, frame_id=self.frame_id), humans=human_msgs))
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FakeHumanPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()