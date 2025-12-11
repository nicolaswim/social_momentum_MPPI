#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import message_filters
import pandas as pd
import os
import sys
import atexit
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MetricsLogger(Node):
    def __init__(self):
        super().__init__("metrics_logger")
        
        # --- Parameters ---
        self.declare_parameter("output_filename", "/home/user/exchange/rosbags/raw/robot_run_data.parquet")
        # INCREASED SLOP: 0.15s might be too tight for Sim. Trying 0.5s.
        self.declare_parameter("slop_seconds", 0.5) 

        self.output_filename = self.get_parameter("output_filename").value
        self.slop_seconds = self.get_parameter("slop_seconds").value
        
        # Print to console so we know where it THINKS it is saving
        print(f"\n[LOGGER] Target File Path: {self.output_filename}\n")

        self.data_buffer = []
        self.joint_name_order = None
        self.joint_velocity_names = None

        # --- Subscribers ---
        # Note: If your robot uses a namespace, these might need to be adjusted.
        # But since your rosbag works with these names, they should be correct.
        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # We pass 'qos_profile=qos_policy' to ensure we can hear the robot
        odom_sub = message_filters.Subscriber(
            self, 
            Odometry, 
            "/mobile_base_controller/odom", 
            qos_profile=qos_policy
        )
        
        joint_sub = message_filters.Subscriber(
            self, 
            JointState, 
            "/joint_states", 
            qos_profile=qos_policy
        )

        # --- Synchronizer ---
        # Allow headerless messages (unlikely for odom/joint_state but safe)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, joint_sub],
            queue_size=50,
            slop=self.slop_seconds,
            allow_headerless=True
        )
        self.ts.registerCallback(self.unified_callback)
        print("[LOGGER] Node initialized. Waiting for synchronized data...")

    def unified_callback(self, odom_msg, joint_msg):
        try:
            # 1. Initialize joint names if needed
            if self.joint_name_order is None:
                self.joint_name_order = joint_msg.name
                self.joint_velocity_names = [n + "_vel" for n in self.joint_name_order]
                print(f"[LOGGER] Joints identified: {len(self.joint_name_order)} joints.")

            # 2. Extract Data
            timestamp = odom_msg.header.stamp
            pos_x = odom_msg.pose.pose.position.x
            pos_y = odom_msg.pose.pose.position.y
            lin_vel = odom_msg.twist.twist.linear.x

            vel_map = dict(zip(joint_msg.name, joint_msg.velocity))
            joint_velocities = [vel_map.get(name, 0.0) for name in self.joint_name_order]

            data_tuple = (timestamp, pos_x, pos_y, lin_vel) + tuple(joint_velocities)
            self.data_buffer.append(data_tuple)

            # 3. --- NEW: AUTO-SAVE EVERY 100 STEPS ---
            # If we have collected 100 new data points, save the file immediately.
            if len(self.data_buffer) % 100 == 0:
                self.save_snapshot()

        except Exception as e:
            print(f"[LOGGER] Error in callback: {e}")

    def save_snapshot(self):
        """Saves the current buffer to disk immediately."""
        try:
            columns = ["timestamp", "pos_x", "pos_y", "lin_vel"] + self.joint_velocity_names
            df = pd.DataFrame(self.data_buffer, columns=columns)
            
            # Convert time
            df["timestamp"] = df["timestamp"].apply(lambda t: t.sec * 1_000_000_000 + t.nanosec)
            
            # Overwrite the file with the latest data
            df.to_parquet(self.output_filename, engine="pyarrow", compression="snappy")
            
            # Print a small dot so you know it's saving without spamming logs
            print(f"[LOGGER] . (Saved {len(df)} rows)", end="", flush=True)
            
        except Exception as e:
            print(f"\n[LOGGER] SAVE FAILED: {e}")

    def save_data_hook(self):
        # We use 'print' here because sometimes the ROS logger dies before this runs
        print(f"\n[LOGGER] Shutdown triggered. Buffer size: {len(self.data_buffer)}")

        if not self.data_buffer:
            print("[LOGGER] !!! ERROR: BUFFER EMPTY. NO DATA COLLECTED. !!!")
            print("[LOGGER] This means topics were not synchronized.")
            print(f"[LOGGER] Checked topics: /mobile_base_controller/odom and /joint_states")
            return

        # Force directory creation just in case
        folder_path = os.path.dirname(self.output_filename)
        if folder_path and not os.path.exists(folder_path):
            try:
                os.makedirs(folder_path, exist_ok=True)
                print(f"[LOGGER] Created directory: {folder_path}")
            except Exception as e:
                print(f"[LOGGER] Failed to create directory: {e}")
                return

        columns = ["timestamp", "pos_x", "pos_y", "lin_vel"] + self.joint_velocity_names
        df = pd.DataFrame(self.data_buffer, columns=columns)
        
        # Convert ROS2 Time (seconds, nanoseconds) to standard integer nanoseconds
        df["timestamp"] = df["timestamp"].apply(lambda t: t.sec * 1_000_000_000 + t.nanosec)

        try:
            df.to_parquet(self.output_filename, engine="pyarrow", compression="snappy")
            print(f"[LOGGER] SUCCESS: Saved {len(df)} rows to:")
            print(f"[LOGGER] >> {self.output_filename}")
        except Exception as e:
            print(f"[LOGGER] CRITICAL FAILURE: Could not save file. {e}")

def main(args=None):
    rclpy.init(args=args)
    logger = MetricsLogger()
    
    # Register hook to run on ANY exit (normal or crash)
    atexit.register(logger.save_data_hook)
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("[LOGGER] KeyboardInterrupt received.")
    finally:
        # cleanup
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()