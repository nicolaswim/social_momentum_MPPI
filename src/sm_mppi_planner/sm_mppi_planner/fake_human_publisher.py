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
        self.declare_parameter('total_people', 5)
        self.declare_parameter('people_standing_up', 3) # NEW: Number of standing/moving humans
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('human_max_speed', 1.5)
        self.declare_parameter('human_min_speed', 0.5)
        self.declare_parameter('world_frame_id', 'odom')
        self.declare_parameter('humans_topic', '/humans')
        self.declare_parameter('markers_topic', '/human_markers')
        self.declare_parameter('x_limits', [-7.0, 7.0])
        self.declare_parameter('y_limits', [-7.0, 7.0])
        self.declare_parameter('standing_human_mesh_path', '') 
        self.declare_parameter('sitting_human_mesh_path', '') 

        self.total_people = self.get_parameter('total_people').value
        self.num_standing = self.get_parameter('people_standing_up').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.max_speed = self.get_parameter('human_max_speed').value
        self.min_speed = self.get_parameter('human_min_speed').value
        self.frame_id = self.get_parameter('world_frame_id').value
        humans_topic = self.get_parameter('humans_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value
        self.standing_mesh_path = self.get_parameter('standing_human_mesh_path').value
        self.sitting_mesh_path = self.get_parameter('sitting_human_mesh_path').value

        # Calculate the number of sitting people
        if self.num_standing > self.total_people:
            self.get_logger().warn("'people_standing_up' is greater than 'total_people'. Setting all people to be standing.")
            self.num_standing = self.total_people
        self.num_sitting = self.total_people - self.num_standing

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
        self.get_logger().info(f"Fake Human Publisher started: {self.num_standing} standing, {self.num_sitting} sitting.")

    def _initialize_humans(self):
        # Initialize STANDING humans
        for i in range(self.num_standing):
            human = {
                'id': f'human_{i}',
                'type': 'standing',
                'pos': np.array([random.uniform(self.x_lim[0], self.x_lim[1]),
                                 random.uniform(self.y_lim[0], self.y_lim[1])]),
                'vel': self.get_random_velocity()*1.5,
                'yaw': 0.0
            }
            self.humans.append(human)

        # Initialize SITTING humans
        for i in range(self.num_standing, self.total_people):
            human = {
                'id': f'human_{i}',
                'type': 'sitting',
                'pos': np.array([random.uniform(self.x_lim[0], self.x_lim[1]),
                                 random.uniform(self.y_lim[0], self.y_lim[1])]),
                'vel': self.get_random_velocity()*0.5,  
                'yaw': 0.0
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
            delete_all_marker = Marker()
            delete_all_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_all_marker)

            for i, human in enumerate(self.humans):
                # --- Update State ---
                human['pos'] += human['vel'] * self.timer_period
                
                # Bounce in bounds logic (applies to all moving humans)
                if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]) or \
                not (self.y_lim[0] < human['pos'][1] < self.y_lim[1]):
                    human['vel'] *= -1.0 
                    # Prevent getting stuck outside bounds
                    human['pos'][0] = np.clip(human['pos'][0], self.x_lim[0], self.x_lim[1])
                    human['pos'][1] = np.clip(human['pos'][1], self.y_lim[0], self.y_lim[1])

                # --- Type-Specific Pose Adjustments ---
                
                # Base orientation from velocity is the same for everyone
                base_yaw = math.atan2(human['vel'][1], human['vel'][0])
                
                # Set default offsets for a standing person
                z_offset = 0.0
                visual_yaw_offset = 0.75 * math.pi 

                # NEW: Overwrite defaults if the human is sitting
                if human['type'] == 'sitting':
                    z_offset = 0.5
                    visual_yaw_offset = math.radians(70.0) # Convert 70 degrees to radians

                # Combine base yaw with the visual offset
                final_yaw = base_yaw + visual_yaw_offset

                # --- Publish TF for the planner ---
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = self.frame_id
                t.child_frame_id = human['id']
                # Apply the z_offset here
                t.transform.translation = Vector3(x=human['pos'][0], y=human['pos'][1], z=z_offset)
                
                # --- Convert Euler angles (roll, pitch, yaw) to a single Quaternion ---
                # Static correction roll to stand the model up (Y-Up to Z-Up)
                roll = math.pi / 2.0 
                pitch = 0.0
                # Use the final_yaw calculated above
                cy = math.cos(final_yaw * 0.5)
                sy = math.sin(final_yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)

                q = Quaternion()
                q.w = cr * cp * cy + sr * sp * sy
                q.x = sr * cp * cy - cr * sp * sy
                q.y = cr * sp * cy + sr * cp * sy
                q.z = cr * cp * sy - sr * sp * cy

                t.transform.rotation = q
                self.tf_broadcaster.sendTransform(t)

                # --- Build the HumanArray message (for other nodes) ---
                hpv_msg = HumanPoseVel()
                hpv_msg.human_id = human['id']
                # Apply the z_offset here too for consistency
                hpv_msg.position = Point(x=human['pos'][0], y=human['pos'][1], z=z_offset)
                hpv_msg.velocity = Vector3(x=human['vel'][0], y=human['vel'][1], z=0.0)
                human_msgs.append(hpv_msg)

                # --- Create the MESH_RESOURCE Marker for RViz ---
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = now
                marker.ns = "human_meshes"
                marker.id = i # Use the index as the unique ID
                marker.action = Marker.ADD
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_use_embedded_materials = True
                
                # Set mesh based on human type
                if human['type'] == 'standing':
                    marker.mesh_resource = self.standing_mesh_path
                else: # 'sitting'
                    marker.mesh_resource = self.sitting_mesh_path
                
                # Set the pose and scale of the human model
                marker.pose.position.x = human['pos'][0]
                marker.pose.position.y = human['pos'][1]
                # Apply the z_offset here for the visual marker
                marker.pose.position.z = z_offset 
                marker.pose.orientation = q
                marker.scale = Vector3(x=1., y=1., z=1.)

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