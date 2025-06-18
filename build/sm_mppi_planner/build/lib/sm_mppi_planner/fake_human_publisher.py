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
        
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        # These limits are passed from your launch file, defining the hallway space
        wall_y_top = self.y_lim[1]
        wall_y_bottom = self.y_lim[0]
        
        for i, human in enumerate(self.humans):
            human['pos'] += human['vel'] * self.timer_period
            
            # --- CORRECTED BOUNCE LOGIC ---

            # 1. Bounce off left/right ends of the hallway (inverts X velocity)
            if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]):
                human['vel'][0] *= -1.0 

            # 2. Bounce off top/bottom hallway walls (inverts Y velocity for a realistic reflection)
            if (human['pos'][1] > wall_y_top and human['vel'][1] > 0) or \
            (human['pos'][1] < wall_y_bottom and human['vel'][1] < 0):
                human['vel'][1] *= -1.0 # This is the key change for realistic bounce

            # Clip position to prevent agents from getting stuck inside the visual wall
            human['pos'][0] = np.clip(human['pos'][0], self.x_lim[0], self.x_lim[1])
            human['pos'][1] = np.clip(human['pos'][1], wall_y_bottom, wall_y_top)

            # --- The rest of your publishing code is unchanged ---
            base_yaw = math.atan2(human['vel'][1], human['vel'][0])
            z_offset = 0.0
            visual_yaw_offset = 0.75 * math.pi
            if human['type'] == 'sitting':
                z_offset = 0.5
                visual_yaw_offset = math.radians(90.0)
            final_yaw = base_yaw + visual_yaw_offset
            
            # This part remains the same to publish TFs and Markers
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = human['id']
            t.transform.translation = Vector3(x=human['pos'][0], y=human['pos'][1], z=z_offset)
            
            roll = math.pi / 2.0
            pitch = 0.0
            cy = math.cos(final_yaw * 0.5); sy = math.sin(final_yaw * 0.5)
            cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)

            q = Quaternion()
            q.w = cr * cp * cy + sr * sp * sy
            q.x = sr * cp * cy - cr * sp * sy
            q.y = cr * sp * cy + sr * cp * sy
            q.z = cr * cp * sy - sr * sp * cy
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

            hpv_msg = HumanPoseVel()
            hpv_msg.human_id = human['id']
            hpv_msg.position = Point(x=human['pos'][0], y=human['pos'][1], z=z_offset)
            hpv_msg.velocity = Vector3(x=human['vel'][0], y=human['vel'][1], z=0.0)
            human_msgs.append(hpv_msg)

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "human_meshes"
            marker.id = i
            marker.action = Marker.ADD
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_use_embedded_materials = True
            
            if human['type'] == 'standing': marker.mesh_resource = self.standing_mesh_path
            else: marker.mesh_resource = self.sitting_mesh_path
            
            marker.pose.position.x = human['pos'][0]
            marker.pose.position.y = human['pos'][1]
            marker.pose.position.z = z_offset
            marker.pose.orientation = q
            marker.scale = Vector3(x=1., y=1., z=1.)
            marker_array.markers.append(marker)

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