# sm_mppi_planner/sm_mppi_planner/human_aggregator_node.py

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from functools import partial

from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from my_social_nav_interfaces.msg import HumanPoseVel, HumanArray

class HumanAggregatorNode(Node):
    def __init__(self):
        super().__init__('human_aggregator_node')
        self.get_logger().info("--- Human Aggregator Node Initializing ---")

        # Declare and get parameters
        self.declare_parameter('trajectory_file', 'default_path.yaml')
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('world_frame_id', 'odom')

        trajectory_file_path = self.get_parameter('trajectory_file').get_parameter_value().string_value
        self.frequency = self.get_parameter('publish_frequency').value
        self.frame_id = self.get_parameter('world_frame_id').value

        # Publishers
        self.human_array_pub = self.create_publisher(HumanArray, '/humans', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/human_markers', 10)

        self.human_states = {}
        self.next_marker_id = 0

        # Load trajectories and create subscribers
        try:
            with open(trajectory_file_path, 'r') as file:
                trajectory_data = yaml.safe_load(file)
                self.get_logger().info(f"Successfully loaded trajectory file from {trajectory_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse trajectory file: {e}")
            return
            
        if 'actors' not in trajectory_data:
            self.get_logger().error("Trajectory file must contain an 'actors' key.")
            return

        for actor in trajectory_data['actors']:
            human_id = actor['name']
            self.human_states[human_id] = {
                'pose': None,
                'velocity': np.array([0.0, 0.0]),
                'last_pos': None,
                'last_time': None
            }
            
            # The topic is published by the gazebo_ros_actor_pose plugin
            actor_topic = f"/actors/{human_id}/pose" 
            
            # Use partial to pass the human_id to the callback
            self.create_subscription(
                PoseStamped,
                actor_topic,
                partial(self.pose_callback, human_id=human_id),
                10
            )
            self.get_logger().info(f"Subscribing to pose for '{human_id}' on topic '{actor_topic}'")

        # Main timer to publish aggregated data
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.publish_data)

        self.get_logger().info("--- Human Aggregator Node Initialized ---")

    def pose_callback(self, msg: PoseStamped, human_id: str):
        """Callback for receiving pose data from a Gazebo actor."""
        current_time = self.get_clock().now()
        current_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        
        state = self.human_states[human_id]
        state['pose'] = msg.pose

        # Calculate velocity
        if state['last_pos'] is not None and state['last_time'] is not None:
            delta_time = (current_time - state['last_time']).nanoseconds / 1e9
            if delta_time > 1e-6: # Avoid division by zero
                delta_pos = current_pos - state['last_pos']
                state['velocity'] = delta_pos / delta_time

        state['last_pos'] = current_pos
        state['last_time'] = current_time

    def publish_data(self):
        """Periodically publishes the HumanArray and MarkerArray."""
        human_msgs = []
        marker_array_msg = MarkerArray()
        current_time_msg = self.get_clock().now().to_msg()
        
        marker_id_counter = 0

        for human_id, state in self.human_states.items():
            if state['pose'] is None:
                continue # Don't publish data for actors not yet detected

            # 1. Create HumanPoseVel message
            hpv_msg = HumanPoseVel()
            hpv_msg.human_id = human_id
            hpv_msg.position = state['pose'].position
            hpv_msg.velocity.x = state['velocity'][0]
            hpv_msg.velocity.y = state['velocity'][1]
            human_msgs.append(hpv_msg)

            # 2. Create Cylinder Marker
            marker = Marker()
            marker.header = Header(stamp=current_time_msg, frame_id=self.frame_id)
            marker.ns = "human_cylinders"
            marker.id = marker_id_counter
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = state['pose']
            marker.pose.position.z = 0.9 # Center the cylinder vertically
            marker.scale = Vector3(x=0.5, y=0.5, z=1.8)
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8) # Cyan color
            marker.lifetime = rclpy.duration.Duration(seconds=self.timer_period * 2.0).to_msg()
            marker_array_msg.markers.append(marker)
            marker_id_counter += 1

            # 3. Create Velocity Arrow Marker
            vel_marker = Marker()
            vel_marker.header = marker.header
            vel_marker.ns = "human_velocities"
            vel_marker.id = marker_id_counter
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD
            
            start_point = Point(x=state['pose'].position.x, y=state['pose'].position.y, z=1.9)
            vel_norm = np.linalg.norm(state['velocity'])
            if vel_norm > 0.1:
                end_point = Point(
                    x=start_point.x + state['velocity'][0],
                    y=start_point.y + state['velocity'][1],
                    z=start_point.z
                )
                vel_marker.points = [start_point, end_point]
                vel_marker.scale = Vector3(x=0.05, y=0.1, z=0.1)
                vel_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
                vel_marker.lifetime = marker.lifetime
                marker_array_msg.markers.append(vel_marker)
            
            marker_id_counter += 1

        # Publish the arrays if they are not empty
        if human_msgs:
            array_msg = HumanArray()
            array_msg.header = Header(stamp=current_time_msg, frame_id=self.frame_id)
            array_msg.humans = human_msgs
            self.human_array_pub.publish(array_msg)
        
        if marker_array_msg.markers:
            self.marker_pub.publish(marker_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanAggregatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()