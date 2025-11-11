#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion, Point
from std_msgs.msg import ColorRGBA

import tf2_ros
from tf2_ros import TransformBroadcaster

from my_social_nav_interfaces.msg import HumanPoseVel, HumanArray

import re
import math

class GazeboActorRelay(Node):
    def __init__(self):
        super().__init__('gazebo_actor_relay')

        # --- Parameters ---
        self.declare_parameter('planner_frame', 'odom')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('unreliable_vel_threshold', 0.01)

        self.planner_frame = self.get_parameter('planner_frame').value
        self.publish_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.vel_threshold = self.get_parameter('unreliable_vel_threshold').value
        
        self.actor_regex = re.compile(r"human\d+|wheelchair\d+")
        self.marker_namespace = "gazebo_actors" # Changed from "human_meshes"
        self.mesh_map = {
            'human': 'package://sm_mppi_planner/models/Slampion/Slampion.dae',
            'wheelchair': 'package://sm_mppi_planner/models/wheelchair/wheelchair.dae'
        }

        # --- Caching for velocity ---
        self.last_pose_cache = {}
        self.last_time_cache = {}
        self.latest_model_states = None

        # --- Gazebo Name to Planner TF Name Mapping ---
        self.gazebo_to_planner_tf_map = {
            "human1": "human_0",
            "human2": "human_1",
            "human3": "human_2",
            "human4": "human_3",
            "human5": "human_4", # This was the wheelchair in scenario_1.world
            "wheelchair1": "human_5", # Example for scenario_2
            # Add all agents you might use
        }
        self.get_logger().info(f"Using TF name map: {self.gazebo_to_planner_tf_map}")


        gazebo_qos_profile = QoSProfile(depth=10) # Use default RELIABLE

        # --- TF Broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Publishers ---
        self.human_array_pub = self.create_publisher(
            HumanArray, 'social_nav/humans', 10 # Renamed from '/humans'
        )
        self.marker_array_pub = self.create_publisher(
            MarkerArray, 'social_nav/human_markers', 10 # Renamed from '/human_markers'
        )

        # --- Subscriber ---
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states', # Correct topic
            self.model_states_callback,
            gazebo_qos_profile
        )

        self.publish_timer = self.create_timer(
            self.publish_period,
            self.process_and_publish
        )
        
        self.get_logger().info(f"Gazebo Actor Relay started. Publishing TF frames and messages to '{self.planner_frame}' frame.")

    def model_states_callback(self, msg):
        self.latest_model_states = msg

    def is_velocity_unreliable(self, vel):
        return abs(vel.x) < self.vel_threshold and \
               abs(vel.y) < self.vel_threshold

    def get_mesh_resource(self, name):
        if name.startswith('human'):
            return self.mesh_map['human']
        if name.startswith('wheelchair'):
            return self.mesh_map['wheelchair']
        return self.mesh_map['human']

    def process_and_publish(self):
        if self.latest_model_states is None:
            return

        msg = self.latest_model_states
        now = self.get_clock().now()

        human_array_msg = HumanArray()
        human_array_msg.header.stamp = now.to_msg()
        human_array_msg.header.frame_id = self.planner_frame
        
        marker_array_msg = MarkerArray()
        transforms_list = []

        clear_marker = Marker()
        clear_marker.header.frame_id = self.planner_frame
        clear_marker.header.stamp = now.to_msg()        # --- Gazebo Name to Planner TF Name Mapping ---
        self.gazebo_to_planner_tf_map = {
            "human1": "human_0",
            "human2": "human_1",
            "human3": "human_2",
            "human4": "human_3",
            "human5": "human_4",
            "human6": "human_5",
            "wheelchair1": "human_6",
            "wheelchair2": "human_7",
        }
        clear_marker.ns = self.marker_namespace
        clear_marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(clear_marker)

        try:
            name_to_index = {name: i for i, name in enumerate(msg.name)}
        except Exception as e:
            self.get_logger().error(f"Error creating name_to_index map: {e}")
            return

        for actor_name in name_to_index.keys():
            if not self.actor_regex.match(actor_name):
                continue
            
            if actor_name not in self.gazebo_to_planner_tf_map:
                if actor_name.startswith("human") or actor_name.startswith("wheelchair"):
                     self.get_logger().warn(f"Actor '{actor_name}' found but has no TF map. Skipping.", throttle_duration_sec=5.0)
                continue
            
            planner_tf_name = self.gazebo_to_planner_tf_map[actor_name]
            index = name_to_index[actor_name]
            actor_pose = msg.pose[index]
            actor_twist = msg.twist[index]
            
            # --- 1. Velocity Calculation ---
            final_velocity = actor_twist.linear
            if self.is_velocity_unreliable(final_velocity):
                if actor_name in self.last_pose_cache:
                    dt_duration = now - self.last_time_cache[actor_name]
                    dt = dt_duration.nanoseconds / 1e9
                    if dt > 1e-3:
                        dx = actor_pose.position.x - self.last_pose_cache[actor_name].position.x
                        dy = actor_pose.position.y - self.last_pose_cache[actor_name].position.y
                        final_velocity = Vector3(x=dx/dt, y=dy/dt, z=0.0)
                else:
                    final_velocity = Vector3(x=0.0, y=0.0, z=0.0)
            self.last_pose_cache[actor_name] = actor_pose
            self.last_time_cache[actor_name] = now

            # --- 2. Build Human Message (HumanArray) ---
            human_msg = HumanPoseVel()
            human_msg.human_id = planner_tf_name
            human_msg.position = actor_pose.position
            human_msg.velocity = final_velocity
            human_array_msg.humans.append(human_msg)

            # --- 3. Build Marker Message (MarkerArray) ---
            # --- THIS SECTION IS NOW COPIED FROM fake_human_publisher.py ---
            marker = Marker()
            marker.header.frame_id = self.planner_frame
            marker.header.stamp = now.to_msg()
            marker.ns = self.marker_namespace # Use the one from this class
            marker.id = index
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_use_embedded_materials = True # <-- THIS IS KEY FOR TEXTURES
            marker.lifetime = Duration(seconds=self.publish_period * 2.0).to_msg()

            # --- Logic from fake_human_publisher ---
            
            # Calculate yaw from velocity (Gazebo's actor pose is often just (0,0,0,1))
            base_yaw = math.atan2(final_velocity.y, final_velocity.x) if (final_velocity.x**2 + final_velocity.y**2)**0.5 > 0.01 else 0.0
            
            z_offset = 0.0
            visual_yaw_offset = 0.75 * math.pi
            
            # Check the *original* Gazebo name
            if actor_name.startswith('wheelchair'): 
                z_offset = 0.5
                visual_yaw_offset = math.radians(90.0)
                marker.mesh_resource = self.mesh_map['wheelchair']
            else:
                marker.mesh_resource = self.mesh_map['human']

            final_yaw = base_yaw + visual_yaw_offset

            # Set marker pose and orientation
            marker.pose.position = Point(x=actor_pose.position.x, y=actor_pose.position.y, z=z_offset)
            
            # Calculate Quaternion from the corrected yaw and roll
            roll = math.pi / 2.0
            pitch = 0.0
            cy=math.cos(final_yaw*0.5); sy=math.sin(final_yaw*0.5)
            cp=math.cos(pitch*0.5); sp=math.sin(pitch*0.5)
            cr=math.cos(roll*0.5); sr=math.sin(roll*0.5)
            q=Quaternion(); q.w=cr*cp*cy+sr*sp*sy; q.x=sr*cp*cy-cr*sp*sy; q.y=cr*sp*cy+sr*cp*sy; q.z=cr*cp*sy-sr*sp*cy
            marker.pose.orientation = q
            
            marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
            
            marker_array_msg.markers.append(marker)
            # --- END OF COPIED LOGIC ---

            # --- 4. Build TF Message (TransformStamped) ---
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.planner_frame
            t.child_frame_id = planner_tf_name
            
            # Use the raw Gazebo pose for TF (planner needs the true position)
            t.transform.translation.x = actor_pose.position.x
            t.transform.translation.y = actor_pose.position.y
            t.transform.translation.z = actor_pose.position.z # Use original Z for TF
            t.transform.rotation = actor_pose.orientation # Use original orientation for TF
            
            transforms_list.append(t)

        # --- Publish everything ---
        self.human_array_pub.publish(human_array_msg)
        self.marker_array_pub.publish(marker_array_msg)
        self.tf_broadcaster.sendTransform(transforms_list)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboActorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()