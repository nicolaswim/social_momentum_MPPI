#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import random
import math
import sys
import os
import yaml

# --- All necessary ROS Imports ---
from geometry_msgs.msg import Point, Pose, Twist, Quaternion, Vector3, TransformStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

# --- Imports for TF Listener ---
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.declare_parameter('startup_delay', 0.0) # <-- ADDED
        self.declare_parameter('humans_yaml', '[]')
        self.declare_parameter('x_limits', [-20.0, 20.0])
        self.declare_parameter('y_limits', [-2.5, 2.5])
        self.declare_parameter('human_radius', 0.3)
        self.declare_parameter('robot_radius', 0.4)
        self.declare_parameter('collision_safety_margin', 0.1)
        self.declare_parameter('sidestep_distance', 0.01)
        self.declare_parameter('world_frame_id', 'odom')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('standing_human_mesh_path', '')
        self.declare_parameter('sitting_human_mesh_path', '')

        # Get parameters
        delay_seconds = self.get_parameter('startup_delay').value # <-- ADDED
        humans_yaml_str = self.get_parameter('humans_yaml').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value
        self.human_radius = self.get_parameter('human_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.collision_threshold = self.human_radius + self.robot_radius + self.get_parameter('collision_safety_margin').value
        self.sidestep_dist = self.get_parameter('sidestep_distance').value
        self.frame_id = self.get_parameter('world_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.standing_mesh_path = self.get_parameter('standing_human_mesh_path').value
        self.sitting_mesh_path = self.get_parameter('sitting_human_mesh_path').value

        # --- TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_pos = np.array([0.0, 0.0])

        # --- Publishers and TF Broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.humans_publisher = self.create_publisher(HumanArray, '/humans', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/human_markers', 10)

        # --- Internal State ---
        self.humans = []
        self.delay_duration = Duration(seconds=delay_seconds) # <-- ADDED
        self.start_time = self.get_clock().now() # <-- ADDED
        self.human_definitions = []
        try:
            self.human_definitions = yaml.safe_load(humans_yaml_str)
            if not isinstance(self.human_definitions, list): self.human_definitions = []
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing 'humans_yaml': {e}")
        
        self._initialize_humans()

        # --- Main Timer ---
        self.timer_period = 1.0 / 10.0
        self.timer = self.create_timer(self.timer_period, self.update_and_publish)
        self.get_logger().info(f"Choreographed Human Publisher started with {len(self.humans)} humans and a {delay_seconds}s delay.")

    def _initialize_humans(self):
        for i, human_def in enumerate(self.human_definitions):
            try:
                human = {
                    'id': f'human_{i}',
                    'type': human_def.get('type', 'standing'),
                    'pos': np.array([float(human_def['x']), float(human_def['y'])]),
                    'vel': np.array([float(human_def['vx']), float(human_def['vy'])])
                }
                self.humans.append(human)
            except (KeyError, TypeError) as e:
                self.get_logger().error(f"Failed to initialize human {i} due to missing/invalid key: {e}")

    def update_and_publish(self):
        # Check if the delay period is active
        is_delay_active = self.get_clock().now() < self.start_time + self.delay_duration

        # Update robot's position from TF
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, self.robot_frame_id, rclpy.time.Time())
            self.robot_pos = np.array([transform.transform.translation.x, transform.transform.translation.y])
        except Exception as e:
            self.get_logger().warn(f"Could not get robot transform: {e}", throttle_duration_sec=2.0)

        human_msgs = []; marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        delete_all_marker = Marker(); delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        wall_y_top = self.y_lim[1] - self.human_radius
        wall_y_bottom = self.y_lim[0] + self.human_radius
        
        for i, human in enumerate(self.humans):
            distance_to_robot = np.linalg.norm(human['pos'] - self.robot_pos)
            is_colliding_with_robot = distance_to_robot < self.collision_threshold
            
            current_tick_velocity = np.array([0.0, 0.0])

            # Apply movement logic only if the initial delay is over
            if not is_delay_active:
                if is_colliding_with_robot:
                    # Sidestep behavior
                    if np.linalg.norm(human['vel']) > 1e-5:
                        direction_vec = human['vel'] / np.linalg.norm(human['vel'])
                    else: 
                        direction_vec = np.array([1.0, 0.0])
                    left_vec = np.array([-direction_vec[1], direction_vec[0]])
                    human['pos'] += left_vec * self.sidestep_dist
                else:
                    # Normal forward movement
                    current_tick_velocity = human['vel']
                    human['pos'] += current_tick_velocity * self.timer_period
            
            # Wall bounce logic (always active)
            if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]):
                human['vel'][0] *= -1.0 
            if (human['pos'][1] > wall_y_top and human['vel'][1] > 0) or \
               (human['pos'][1] < wall_y_bottom and human['vel'][1] < 0):
                human['vel'][1] *= -1.0

            human['pos'][0] = np.clip(human['pos'][0], self.x_lim[0], self.x_lim[1])
            human['pos'][1] = np.clip(human['pos'][1], self.y_lim[0], self.y_lim[1])

            # ORIENTATION & VISUALIZATION
            base_yaw = math.atan2(human['vel'][1], human['vel'][0]) if np.linalg.norm(human['vel']) > 0.01 else 0.0
            z_offset=0.0; visual_yaw_offset=0.75*math.pi
            if human['type']=='sitting': z_offset=0.5; visual_yaw_offset=math.radians(90.0)
            final_yaw = base_yaw + visual_yaw_offset
            
            t=TransformStamped(); t.header.stamp=now; t.header.frame_id=self.frame_id; t.child_frame_id=human['id']
            t.transform.translation=Vector3(x=human['pos'][0], y=human['pos'][1], z=z_offset)
            roll=math.pi/2.0; pitch=0.0
            cy=math.cos(final_yaw*0.5); sy=math.sin(final_yaw*0.5); cp=math.cos(pitch*0.5); sp=math.sin(pitch*0.5); cr=math.cos(roll*0.5); sr=math.sin(roll*0.5)
            q=Quaternion(); q.w=cr*cp*cy+sr*sp*sy; q.x=sr*cp*cy-cr*sp*sy; q.y=cr*sp*cy+sr*cp*sy; q.z=cr*cp*sy-sr*sp*cy
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

            hpv_msg=HumanPoseVel(); hpv_msg.human_id=human['id']; hpv_msg.position=Point(x=human['pos'][0],y=human['pos'][1],z=z_offset)
            hpv_msg.velocity=Vector3(x=current_tick_velocity[0],y=current_tick_velocity[1],z=0.0)
            human_msgs.append(hpv_msg)

            marker=Marker(); marker.header.frame_id=self.frame_id; marker.header.stamp=now
            marker.ns="human_meshes"; marker.id=i; marker.action=Marker.ADD; marker.type=Marker.MESH_RESOURCE; marker.mesh_use_embedded_materials=True
            if human['type']=='standing': marker.mesh_resource=self.standing_mesh_path
            else: marker.mesh_resource=self.sitting_mesh_path
            marker.pose.position=Point(x=human['pos'][0], y=human['pos'][1], z=z_offset); marker.pose.orientation=q; marker.scale=Vector3(x=1.,y=1.,z=1.)
            marker_array.markers.append(marker)

        self.humans_publisher.publish(HumanArray(header=Header(stamp=now, frame_id=self.frame_id), humans=human_msgs))
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = FakeHumanPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()