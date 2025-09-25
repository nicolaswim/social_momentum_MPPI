#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import sys
import yaml
from rclpy.duration import Duration
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped

class GazeboHumanController(Node):
    """
    This node controls the position of spawned models within the Gazebo simulation.
    It reads a choreographed plan from a YAML string and updates each model's pose
    at a regular interval by calling the /gazebo/set_entity_state service.
    """
    def __init__(self):
        super().__init__('gazebo_human_controller')
        
        # Parameters
        self.declare_parameter('humans_yaml', '[]')
        self.declare_parameter('world_frame_id', 'odom')
        self.declare_parameter('x_limits', [-20.0, 20.0])
        self.declare_parameter('y_limits', [-2.5, 2.5])
        self.declare_parameter('human_radius', 0.3)
        self.declare_parameter('startup_delay', 0.0)

        # Get parameters
        humans_yaml_str = self.get_parameter('humans_yaml').value
        delay_seconds = self.get_parameter('startup_delay').value
        self.frame_id = self.get_parameter('world_frame_id').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value
        self.human_radius = self.get_parameter('human_radius').value

        # Gazebo service client
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Gazebo\'s /set_entity_state service not available, waiting...')

        # Internal State
        self.humans = []
        self.delay_duration = Duration(seconds=delay_seconds)
        self.start_time = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)

        try:
            human_definitions = yaml.safe_load(humans_yaml_str)
            if not isinstance(human_definitions, list): human_definitions = []
            self._initialize_humans(human_definitions)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing 'humans_yaml': {e}")
        
        # Main Timer
        self.timer_period = 1.0 / 10.0
        self.timer = self.create_timer(self.timer_period, self.update_human_poses)
        self.get_logger().info(f"Gazebo Human Controller started with {len(self.humans)} humans and a {delay_seconds}s delay.")

    def _initialize_humans(self, definitions):
        """Initializes humans from the parsed YAML data."""
        for i, human_def in enumerate(definitions):
            try:
                human = {
                    'name': f'human_{i}',
                    'pos': np.array([float(human_def['x']), float(human_def['y'])]),
                    'vel': np.array([float(human_def['vx']), float(human_def['vy'])])
                }
                self.humans.append(human)
            except (KeyError, TypeError) as e:
                self.get_logger().error(f"Failed to initialize human {i}: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy=math.cos(yaw*0.5); sy=math.sin(yaw*0.5); cp=math.cos(pitch*0.5); sp=math.sin(pitch*0.5); cr=math.cos(roll*0.5); sr=math.sin(roll*0.5)
        return Quaternion(x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy, w=cr*cp*cy+sr*sp*sy)

    def update_human_poses(self):
        """Calculates new positions and calls Gazebo service to move the models."""
        if self.get_clock().now() < self.start_time + self.delay_duration:
            return

        wall_y_top = self.y_lim[1] - self.human_radius
        wall_y_bottom = self.y_lim[0] + self.human_radius
        
        for human in self.humans:
            human['pos'] += human['vel'] * self.timer_period
            
            if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]):
                human['vel'][0] *= -1.0 
            if (human['pos'][1] > wall_y_top and human['vel'][1] > 0) or \
               (human['pos'][1] < wall_y_bottom and human['vel'][1] < 0):
                human['vel'][1] *= -1.0
            
            human['pos'][0] = np.clip(human['pos'][0], self.x_lim[0], self.x_lim[1])
            human['pos'][1] = np.clip(human['pos'][1], self.y_lim[0], self.y_lim[1])

            self.set_model_pose(human['name'], human['pos'], human['vel'])

    def set_model_pose(self, name, pos, vel):
        yaw = math.atan2(vel[1], vel[0]) if np.linalg.norm(vel) > 0.01 else 0.0
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position = Point(x=pos[0], y=pos[1], z=0.0)
        req.state.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
        self.set_state_client.call_async(req)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = name
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.rotation = req.state.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboHumanController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()