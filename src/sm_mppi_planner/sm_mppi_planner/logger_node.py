#!/usr/bin/env python3

## Component 1: Real-Time Synchronous Logger (ROS 2 / rclpy)
##
## This ROS 2 node subscribes to odometry and joint states.
## It uses ApproximateTimeSynchronizer to create a unified data stream.
## Data is buffered *in-memory* as a list of tuples for maximum performance.
##
## On shutdown (Ctrl+C), the `finally` block catches the KeyboardInterrupt
## and fires the save_data_hook, converting the buffer to a Pandas 
## DataFrame and saving it to a single, highly-compressed Parquet file.
##

import rclpy
from rclpy.node import Node
import message_filters
import pandas as pd
import os
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
# BatteryState import removed

class MetricsLogger(Node):
    def __init__(self):
        super().__init__("metrics_logger")
        self.get_logger().info("Initializing Metrics Logger Node...")

        # --- Parameters ---
        self.declare_parameter("output_filename", "robot_run_data.parquet")
        self.declare_parameter("slop_seconds", 0.15)

        self.output_filename = self.get_parameter("output_filename").value
        self.slop_seconds = self.get_parameter("slop_seconds").value
        
        self.get_logger().info(f"Output file: {self.output_filename}")
        self.get_logger().info(f"Time slop: {self.slop_seconds}s")

        # --- Data Storage ---
        self.data_buffer = []

        # --- JointState Management ---
        self.joint_name_order = None
        self.joint_velocity_names = None

        # --- ROS 2 Subscribers (using message_filters) ---
        odom_sub = message_filters.Subscriber(self, Odometry, "/mobile_base_controller/odom") # <-- FIXED
        joint_sub = message_filters.Subscriber(self, JointState, "/joint_states")
        # battery_sub removed

        # Create the ApproximateTimeSynchronizer
        # --- MODIFIED: Now only synchronizes two topics ---
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, joint_sub],
            queue_size=10,
            slop=self.slop_seconds,
        )

        # Register the single, unified callback
        self.ts.registerCallback(self.unified_callback)

        self.get_logger().info("Metrics Logger is running. Buffering data in memory...")

    # --- MODIFIED: Callback signature changed ---
    def unified_callback(self, odom_msg, joint_msg):
        """
        The single, time-synchronized callback.
        Deconstructs ROS messages and appends a lightweight tuple to
        the in-memory buffer.
        """
        try:
            # --- One-time joint name initialization ---
            if self.joint_name_order is None:
                self.joint_name_order = joint_msg.name
                self.joint_velocity_names = [
                    name + "_vel" for name in self.joint_name_order
                ]
                self.get_logger().info(
                    f"Joints initialized. Logging {len(self.joint_name_order)} joints."
                )

            # 1. Get timestamp (use odom as the reference)
            timestamp = odom_msg.header.stamp

            # 2. Extract Odometry data
            pos_x = odom_msg.pose.pose.position.x
            pos_y = odom_msg.pose.pose.position.y
            lin_vel = odom_msg.twist.twist.linear.x

            # 3. Extract JointState data
            vel_map = dict(zip(joint_msg.name, joint_msg.velocity))
            joint_velocities = [vel_map[name] for name in self.joint_name_order]

            # 4. Power data (REMOVED)

            # 5. Create the data tuple
            # --- MODIFIED: 'power' removed from tuple ---
            data_tuple = (
                timestamp,
                pos_x,
                pos_y,
                lin_vel,
            ) + tuple(joint_velocities)

            # 6. Append to the in-memory buffer
            self.data_buffer.append(data_tuple)

        except Exception as e:
            self.get_logger().warn(f"Error in callback: {e}")

    def save_data_hook(self):
        """
        This function is called *only* on node shutdown.
        It performs the high-speed dump of the in-memory buffer to disk.
        """
        self.get_logger().info("Shutdown hook activated.")
        
        if not self.data_buffer:
            self.get_logger().info("No data collected. Exiting.")
            return

        self.get_logger().info(
            f"Converting in-memory buffer ({len(self.data_buffer)} rows) to DataFrame..."
        )

        # 1. Define the DataFrame columns
        # --- MODIFIED: 'power_w' removed from columns ---
        columns = [
            "timestamp",
            "pos_x",
            "pos_y",
            "lin_vel",
        ] + self.joint_velocity_names

        # 2. Create the Pandas DataFrame
        df = pd.DataFrame(self.data_buffer, columns=columns)

        # 3. Convert ROS 2 timestamp to pandas datetime
        df["timestamp"] = pd.to_datetime(
            df["timestamp"].apply(lambda ts: (ts.sec * 1e9) + ts.nanosec)
        )

        self.get_logger().info(f"Saving DataFrame to {self.output_filename}...")

        try:
            # 4. Save to Parquet
            df.to_parquet(
                self.output_filename, engine="pyarrow", compression="snappy"
            )
            self.get_logger().info(
                f"Successfully saved data to {self.output_filename}."
            )

        except Exception as e:
            self.get_logger().error(f"CRITICAL: Failed to save data to disk: {e}")
            self.get_logger().error("Data in buffer is LOST.")


def main(args=None):
    rclpy.init(args=args)
    
    logger = MetricsLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        # This block executes *after* spin() returns,
        logger.save_data_hook()
        logger.destroy_node()
        # rclpy.shutdown() # Removed this, launch system handles it

if __name__ == "__main__":
    main()