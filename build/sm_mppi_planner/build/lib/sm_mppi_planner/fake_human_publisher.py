#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import random
import math
import sys
import os

# --- GAZEBO & ROS Imports ---
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Twist, Quaternion, Vector3, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.srv import SpawnEntity, SetEntityState, DeleteEntity
from gazebo_msgs.msg import EntityState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# --- Your Custom Interface ---
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
        self.declare_parameter('scenario_mode', 'random')
        self.declare_parameter('num_random_humans', 5)
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('human_max_speed', 1.5)
        self.declare_parameter('human_min_speed', 0.5)
        self.declare_parameter('world_frame_id', 'odom')
        self.declare_parameter('humans_topic', '/humans')
        self.declare_parameter('markers_topic', '/human_markers')
        self.declare_parameter('x_limits', [-7.0, 7.0])
        self.declare_parameter('y_limits', [-7.0, 7.0])
        self.declare_parameter('teleop_cmd_topic', '/human_teleop_cmd_vel')

        self.scenario = self.get_parameter('scenario_mode').value
        self.num_random = self.get_parameter('num_random_humans').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.max_speed = self.get_parameter('human_max_speed').value
        self.min_speed = self.get_parameter('human_min_speed').value
        self.frame_id = self.get_parameter('world_frame_id').value
        humans_topic = self.get_parameter('humans_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        teleop_cmd_topic = self.get_parameter('teleop_cmd_topic').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value

        # --- Gazebo service clients ---
        self.get_logger().info("Connecting to Gazebo services...")
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gazebo /spawn_entity service not available.")
            sys.exit(1)
        self.get_logger().info("Gazebo services connected.")

        # --- TF broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Publishers & Subscribers ---
        self.publisher_ = self.create_publisher(HumanArray, humans_topic, 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, markers_topic, 10)
        if self.scenario == 'teleop':
            self.teleop_sub = self.create_subscription(
                Twist, teleop_cmd_topic, self.teleop_cmd_callback, 10)
        else:
            self.teleop_sub = None

        # --- Internal state ---
        self.humans = []
        self.teleop_velocity = np.array([0.0, 0.0])
        self.sdf_xml = ""

        # --- Main timer ---
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.update_and_publish)

        # Load SDF for spawning
        self.load_sdf_file()
        # Initialize and spawn
        self._initialize_humans()

    def load_sdf_file(self):
        # Using a simple cylinder model for Gazebo to match RViz
        # THE FIX: Added <kinematic>true</kinematic> to the link
        self.sdf_xml = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="human_cylinder">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="link">
      <kinematic>true</kinematic>
      <collision name="collision">
        <geometry><cylinder><radius>0.2</radius><length>1.0</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.2</radius><length>1.0</length></cylinder></geometry>
        <material><diffuse>0 1 0 0.8</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _initialize_humans(self):
        self.get_logger().info(f"Initializing '{self.scenario}' scenario...")
        
        if self.scenario == 'head_on':
            start_x = -self.x_lim[0] + 0.1
            start_y = 0.0
            speed = -random.uniform(self.min_speed, self.max_speed)
            human = {
                'id': 'human_0',
                'pos': np.array([start_x, start_y]),
                'vel': np.array([speed, 0.0])
            }
            self.humans.append(human)
            pose = Pose(position=Point(x=start_x, y=start_y, z=0.0))
            self.spawn_model(human['id'], pose)

        elif self.scenario == 'random':
            for i in range(self.num_random):
                human_id = f'human_{i}'
                pos_x = random.uniform(self.x_lim[0], self.x_lim[1])
                pos_y = random.uniform(self.y_lim[0], self.y_lim[1])
                speed = random.uniform(self.min_speed, self.max_speed)
                angle = random.uniform(0, 2 * math.pi)
                vel_x = speed * math.cos(angle)
                vel_y = speed * math.sin(angle)
                human = {
                    'id': human_id,
                    'pos': np.array([pos_x, pos_y]),
                    'vel': np.array([vel_x, vel_y])
                }
                self.humans.append(human)
                pose = Pose(position=Point(x=pos_x, y=pos_y, z=0.0))
                self.spawn_model(human['id'], pose)

        elif self.scenario == 'teleop':
            human_id = 'human_teleop_0'
            start_x = 0.0
            start_y = (self.y_lim[0] + self.y_lim[1]) / 2.0
            human = {
                'id': human_id,
                'pos': np.array([start_x, start_y]),
                'vel': np.array([0.0, 0.0])
            }
            self.humans.append(human)
            pose = Pose(position=Point(x=start_x, y=start_y, z=0.0))
            self.spawn_model(human['id'], pose)
        else:
            self.get_logger().error(f"Unknown scenario mode in _initialize_humans: {self.scenario}")

    def spawn_model(self, model_name: str, initial_pose: Pose):
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = self.sdf_xml.replace('human_cylinder', model_name)
        req.initial_pose = initial_pose
        fut = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res and res.success:
            self.get_logger().info(f"Spawned '{model_name}' in Gazebo.")
        else:
            err = res.status_message if res else 'no response'
            self.get_logger().error(f"Failed to spawn '{model_name}': {err}")

    def teleop_cmd_callback(self, msg: Twist):
        if self.humans and self.scenario == 'teleop':
            self.teleop_velocity = np.array([msg.linear.x, msg.linear.y])
            self.humans[0]['vel'] = self.teleop_velocity

    def update_and_publish(self):
        dt = self.timer_period
        human_msgs = []
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, human in enumerate(self.humans):
            # Update position from velocity
            human['pos'] += human['vel'] * dt
            # Bounce in bounds logic
            if self.scenario == 'random' or self.scenario == 'head_on':
                if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]):
                    human['vel'][0] *= -1.0
                if not (self.y_lim[0] < human['pos'][1] < self.y_lim[1]):
                    human['vel'][1] *= -1.0

            # Broadcast TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = human['id']
            t.transform.translation = Vector3(
                x=human['pos'][0], y=human['pos'][1], z=0.0)
            yaw = math.atan2(human['vel'][1], human['vel'][0])
            t.transform.rotation = Quaternion(z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))
            self.tf_broadcaster.sendTransform(t)

            # Set Gazebo entity state
            state_req = SetEntityState.Request()
            state = EntityState()
            state.name = human['id']
            state.pose.position = Point(
                x=human['pos'][0], y=human['pos'][1], z=0.5)
            state.pose.orientation = t.transform.rotation
            state.twist.linear = Vector3(x=human['vel'][0], y=human['vel'][1], z=0.0)
            state.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
            state_req.state = state
            self.set_state_client.call_async(state_req)

            # Build HumanArray message for other nodes
            msg = HumanPoseVel()
            msg.human_id = human['id']
            msg.position = Point(
                x=human['pos'][0], y=human['pos'][1], z=0.0)
            msg.velocity = Vector3(
                x=human['vel'][0], y=human['vel'][1], z=0.0)
            human_msgs.append(msg)

            # Build RViz Markers
            # Cylinder Marker
            cyl = Marker()
            cyl.header.frame_id = self.frame_id
            cyl.header.stamp = now
            cyl.ns = 'human_cyl'
            cyl.id = idx * 2
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position = state.pose.position
            cyl.pose.orientation = Quaternion(w=1.0)
            cyl.scale = Vector3(x=0.4, y=0.4, z=1.0)
            cyl.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            cyl.lifetime = rclpy.duration.Duration(seconds=self.timer_period*2).to_msg()
            markers.markers.append(cyl)

            # Velocity Arrow Marker
            if np.linalg.norm(human['vel']) > 0.01:
                arr = Marker()
                arr.header = cyl.header
                arr.ns = 'human_vel'
                arr.id = idx * 2 + 1
                arr.type = Marker.ARROW
                arr.action = Marker.ADD
                start = Point(x=human['pos'][0], y=human['pos'][1], z=1.1)
                end = Point(x=start.x + human['vel'][0]*0.5,
                            y=start.y + human['vel'][1]*0.5, z=start.z)
                arr.points = [start, end]
                arr.scale = Vector3(x=0.05, y=0.1, z=0.1)
                arr.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                arr.lifetime = cyl.lifetime
                markers.markers.append(arr)

        # Publish both topics
        array = HumanArray(header=Header(stamp=now, frame_id=self.frame_id), humans=human_msgs)
        self.publisher_.publish(array)
        self.marker_publisher_.publish(markers)

    def on_shutdown(self):
        self.get_logger().info("Deleting all spawned humans from Gazebo...")
        for human in self.humans:
            req = DeleteEntity.Request()
            req.name = human['id']
            future = self.delete_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeHumanPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt. Shutting down cleanly.")
    finally:
        if node:
            node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()