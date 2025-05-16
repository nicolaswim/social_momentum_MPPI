#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import random
import math
import time # For initial delay
import sys 
import os 

from geometry_msgs.msg import Point, Vector3, Twist, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped



try:
    from my_social_nav_interfaces.msg import HumanPoseVel, HumanArray
except ImportError:
    print("ERROR: Cannot import custom messages from my_social_nav_interfaces.", file=sys.stderr)
    print("Ensure the interface package is built and the current workspace is sourced.", file=sys.stderr)
    sys.exit(1)


class FakeHumanPublisher(Node):
    def __init__(self):
        super().__init__('fake_human_publisher')
        self.get_logger().info("--- FakeHumanPublisher Node Initializing ---")

        self.declare_parameter('scenario_mode', 'random') 
        self.declare_parameter('num_random_humans', 3)    
        self.declare_parameter('publish_frequency', 10.0) 
        self.declare_parameter('human_max_speed', 1.5)    
        self.declare_parameter('human_min_speed', 0.5)    
        self.declare_parameter('world_frame_id', 'odom')  
        self.declare_parameter('humans_topic', '/humans') 
        self.declare_parameter('markers_topic', '/human_markers') 
        self.declare_parameter('x_limits', [-5.0, 5.0])
        self.declare_parameter('y_limits', [-5.0, 5.0])
        self.declare_parameter('teleop_cmd_topic', '/human_teleop_cmd_vel')
        self.declare_parameter('initial_delay_sec', 1.0) # Added for clarity

        self.scenario = self.get_parameter('scenario_mode').value
        self.num_random = self.get_parameter('num_random_humans').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.max_speed = self.get_parameter('human_max_speed').value
        self.min_speed = self.get_parameter('human_min_speed').value
        self.frame_id = self.get_parameter('world_frame_id').value
        humans_topic = self.get_parameter('humans_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value
        teleop_cmd_topic = self.get_parameter('teleop_cmd_topic').value
        self.initial_delay_sec = self.get_parameter('initial_delay_sec').value

        self.tf_broadcaster = TransformBroadcaster(self)


        self.get_logger().info(f"Scenario: '{self.scenario}', Frame: '{self.frame_id}'")

        self.humans = [] 
        self.teleop_velocity = np.array([0.0, 0.0])
        self.next_marker_id = 0 # For ensuring unique marker IDs

        self.get_logger().info("Initializing humans...")
        self._initialize_humans()
        self.get_logger().info(f"Humans initialized: {len(self.humans)} humans")

        self.publisher_ = self.create_publisher(HumanArray, humans_topic, 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, markers_topic, 10)
        self.get_logger().info("Publishers created.")

        if self.scenario == 'teleop':
            self.teleop_sub = self.create_subscription(
                Twist,
                teleop_cmd_topic,
                self.teleop_cmd_callback,
                10)
            self.get_logger().info(f"Teleop subscriber created for {teleop_cmd_topic}")
        else:
             self.teleop_sub = None

        self.timer_period = 1.0 / self.frequency
        
        # Apply initial delay before starting the main timer
        self.get_logger().info(f"Waiting for initial delay of {self.initial_delay_sec} seconds before starting publisher timer.")
        self.initial_delay_timer = self.create_timer(self.initial_delay_sec, self._start_main_timer)
        self.main_publisher_timer = None # Will be created after delay

        self.get_logger().info("--- FakeHumanPublisher Initialization Complete ---")

    def _start_main_timer(self):
        """Callback to start the main publishing timer after the initial delay."""
        self.get_logger().info("Initial delay complete. Starting main publisher timer.")
        if self.initial_delay_timer:
            self.initial_delay_timer.cancel() # Cancel the delay timer
        self.main_publisher_timer = self.create_timer(self.timer_period, self.update_and_publish)


    def _initialize_humans(self):
        self.humans = []
        if self.scenario == 'head_on':
            start_x = -self.x_lim[0] + 0.1  # start on the left
            start_y = 0.0
            speed = -random.uniform(self.min_speed, self.max_speed)
            human = {
                'id': 'human_1',
                'marker_base_id': self.next_marker_id,
                'pos': np.array([start_x, start_y]),
                'vel': np.array([speed, 0.0])  # move along +x axis
            }
            self.humans.append(human)
            self.next_marker_id += 2


        elif self.scenario == 'random':
            for i in range(self.num_random):
                pos_x = random.uniform(self.x_lim[0], self.x_lim[1])
                pos_y = random.uniform(self.y_lim[0], self.y_lim[1])
                speed = random.uniform(self.min_speed, self.max_speed)
                angle = random.uniform(0, 2 * math.pi)
                vel_x = speed * math.cos(angle)
                vel_y = speed * math.sin(angle)
                human = {
                    'id': f'human_{i}',
                    'marker_base_id': self.next_marker_id,
                    'pos': np.array([pos_x, pos_y]),
                    'vel': np.array([vel_x, vel_y])
                }
                self.humans.append(human)
                self.next_marker_id += 2 

        elif self.scenario == 'teleop':
            start_x = 0.0
            start_y = (self.y_lim[0] + self.y_lim[1]) / 2.0 
            human = {
                'id': 'human_teleop_0',
                'marker_base_id': self.next_marker_id,
                'pos': np.array([start_x, start_y]),
                'vel': np.array([0.0, 0.0]) 
            }
            self.humans.append(human)
            self.next_marker_id += 2
        else:
            self.get_logger().error(f"Unknown scenario mode in _initialize_humans: {self.scenario}")

    def teleop_cmd_callback(self, msg: Twist):
         if self.scenario == 'teleop' and self.humans:
             self.teleop_velocity = np.array([msg.linear.x, msg.linear.y])
             self.humans[0]['vel'] = self.teleop_velocity

    def update_and_publish(self):
        dt = self.timer_period
        human_msgs = []
        marker_array_msg = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        if not self.humans:
            return 

        for human_idx, human in enumerate(self.humans):
            human['pos'] = human['pos'] + human['vel'] * dt
            if self.scenario == 'random':
                if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]):
                    human['vel'][0] *= -1 
                    human['pos'][0] = np.clip(human['pos'][0], self.x_lim[0]+0.1, self.x_lim[1]-0.1)
                if not (self.y_lim[0] < human['pos'][1] < self.y_lim[1]):
                    human['vel'][1] *= -1 
                    human['pos'][1] = np.clip(human['pos'][1], self.y_lim[0]+0.1, self.y_lim[1]-0.1)

            # Publish TF transform for each human
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = human['id']
            t.transform.translation.x = float(human['pos'][0])
            t.transform.translation.y = float(human['pos'][1])
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

            # Fill custom message
            hpv_msg = HumanPoseVel()
            hpv_msg.human_id = human['id']
            hpv_msg.position.x = human['pos'][0]
            hpv_msg.position.y = human['pos'][1]
            hpv_msg.position.z = 0.0
            hpv_msg.velocity.x = human['vel'][0]
            hpv_msg.velocity.y = human['vel'][1]
            hpv_msg.velocity.z = 0.0
            human_msgs.append(hpv_msg)

            # Marker
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = current_time
            marker.ns = "human_cylinders" 
            marker.id = human['marker_base_id']
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = human['pos'][0]
            marker.pose.position.y = human['pos'][1]
            marker.pose.position.z = 0.5 
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale = Vector3(x=0.4, y=0.4, z=1.0) 
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) 
            marker.lifetime = rclpy.duration.Duration(seconds=self.timer_period * 2.5).to_msg()
            marker_array_msg.markers.append(marker)

            # Velocity arrow
            vel_marker = Marker()
            vel_marker.header = marker.header 
            vel_marker.ns = "human_velocities"
            vel_marker.id = human['marker_base_id'] + 1
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD

            start_point = Point(x=human['pos'][0], y=human['pos'][1], z=1.1) 
            vel_scale = 0.5 
            if np.linalg.norm(human['vel']) > 0.1:
                end_point = Point(
                    x=start_point.x + human['vel'][0] * vel_scale,
                    y=start_point.y + human['vel'][1] * vel_scale,
                    z=start_point.z 
                )
                vel_marker.points = [start_point, end_point]
                vel_marker.scale = Vector3(x=0.05, y=0.1, z=0.15) 
                vel_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) 
                vel_marker.lifetime = marker.lifetime
                marker_array_msg.markers.append(vel_marker)
            else:
                vel_marker.action = Marker.DELETE 
                marker_array_msg.markers.append(vel_marker)

        array_msg = HumanArray()
        array_msg.header = Header(stamp=current_time, frame_id=self.frame_id)
        array_msg.humans = human_msgs
        self.publisher_.publish(array_msg)

        if marker_array_msg.markers:
            self.marker_publisher_.publish(marker_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeHumanPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("FakeHumanPublisher KeyboardInterrupt.")
    except Exception as e:
        if node:
             node.get_logger().error(f"Exception in FakeHumanPublisher: {e}")
             import traceback
             node.get_logger().error(traceback.format_exc())
        else:
             print(f"Exception before FakeHumanPublisher node initialization: {e}", file=sys.stderr)
             import traceback
             traceback.print_exc()
    finally:
        if node and rclpy.ok():
             node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        if node: print("FakeHumanPublisher shutdown complete.")

if __name__ == '__main__':
    main()
