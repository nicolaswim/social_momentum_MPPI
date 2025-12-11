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
        # HARDCODED PATH (As requested for safety)
        self.declare_parameter("output_filename", "/home/user/exchange/rosbags/raw/robot_run_data.parquet")
        
        # Increased slop for simulation sync issues
        self.declare_parameter("slop_seconds", 0.5) 

        self.output_filename = self.get_parameter("output_filename").value
        self.slop_seconds = self.get_parameter("slop_seconds").value
        
        # =========================================================
        # SANITY CHECK: CAN WE WRITE TO THE DISK?
        # =========================================================
        print(f"\n[LOGGER] --- DIAGNOSTIC CHECK START ---", flush=True)
        print(f"[LOGGER] Target Directory: {os.path.dirname(self.output_filename)}", flush=True)
        
        try:
            # 1. Create directory if it doesn't exist
            folder_path = os.path.dirname(self.output_filename)
            os.makedirs(folder_path, exist_ok=True)
            
            # 2. Write a dummy text file
            test_file_path = os.path.join(folder_path, "test_write_check.txt")
            with open(test_file_path, "w") as f:
                f.write("SUCCESS! The container has permission to write to this folder.")
                
            print(f"[LOGGER] SUCCESS: Created test file at:", flush=True)
            print(f"[LOGGER] >> {test_file_path}", flush=True)
            print(f"[LOGGER] PLEASE CHECK YOUR WINDOWS/HOST FOLDER NOW FOR 'test_write_check.txt'", flush=True)
            
        except Exception as e:
            print(f"[LOGGER] !!! FAILURE !!! Could not write to disk.", flush=True)
            print(f"[LOGGER] Error details: {e}", flush=True)
        print(f"[LOGGER] --- DIAGNOSTIC CHECK END ---\n", flush=True)
        # =========================================================

        self.data_buffer = []
        self.joint_name_order = None
        self.joint_velocity_names = None

        # --- Subscribers (QoS FIXED for Best Effort) ---
        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        print("[LOGGER] Subscribing with BEST_EFFORT reliability...", flush=True)

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
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, joint_sub],
            queue_size=50,
            slop=self.slop_seconds,
            allow_headerless=True
        )
        self.ts.registerCallback(self.unified_callback)
        print("[LOGGER] Node initialized. Waiting for data...", flush=True)

    def unified_callback(self, odom_msg, joint_msg):
        try:
            # 1. Initialize joint names if needed
            if self.joint_name_order is None:
                self.joint_name_order = joint_msg.name
                self.joint_velocity_names = [n + "_vel" for n in self.joint_name_order]
                print(f"[LOGGER] Joints identified: {len(self.joint_name_order)} joints.", flush=True)

            # 2. Extract Data
            timestamp = odom_msg.header.stamp
            pos_x = odom_msg.pose.pose.position.x
            pos_y = odom_msg.pose.pose.position.y
            lin_vel = odom_msg.twist.twist.linear.x

            vel_map = dict(zip(joint_msg.name, joint_msg.velocity))
            joint_velocities = [vel_map.get(name, 0.0) for name in self.joint_name_order]

            data_tuple = (timestamp, pos_x, pos_y, lin_vel) + tuple(joint_velocities)
            self.data_buffer.append(data_tuple)

            # 3. Auto-save periodically
            if len(self.data_buffer) % 100 == 0:
                self.save_snapshot()

        except Exception as e:
            print(f"[LOGGER] Error in callback: {e}", flush=True)

    def save_snapshot(self):
        """Saves the current buffer to disk immediately."""
        try:
            columns = ["timestamp", "pos_x", "pos_y", "lin_vel"] + self.joint_velocity_names
            df = pd.DataFrame(self.data_buffer, columns=columns)
            
            # Convert time
            df["timestamp"] = df["timestamp"].apply(lambda t: t.sec * 1_000_000_000 + t.nanosec)
            
            df.to_parquet(self.output_filename, engine="pyarrow", compression="snappy")
            
            # Use flush=True to force the print to appear
            print(f"[LOGGER] . (Saved {len(df)} rows)", end="", flush=True)
            
        except Exception as e:
            print(f"\n[LOGGER] SAVE FAILED: {e}", flush=True)

    def save_data_hook(self):
        print(f"\n[LOGGER] Shutdown triggered. Buffer size: {len(self.data_buffer)}", flush=True)

        if not self.data_buffer:
            print("[LOGGER] !!! BUFFER EMPTY. NO DATA COLLECTED !!!", flush=True)
            return

        # Double check directory exists
        folder_path = os.path.dirname(self.output_filename)
        if folder_path and not os.path.exists(folder_path):
            try:
                os.makedirs(folder_path, exist_ok=True)
            except Exception as e:
                print(f"[LOGGER] Failed to create dir: {e}", flush=True)
                return

        columns = ["timestamp", "pos_x", "pos_y", "lin_vel"] + self.joint_velocity_names
        df = pd.DataFrame(self.data_buffer, columns=columns)
        df["timestamp"] = df["timestamp"].apply(lambda t: t.sec * 1_000_000_000 + t.nanosec)

        try:
            df.to_parquet(self.output_filename, engine="pyarrow", compression="snappy")
            print(f"[LOGGER] SUCCESS: Final save complete to {self.output_filename}", flush=True)
        except Exception as e:
            print(f"[LOGGER] CRITICAL FAILURE: Could not save file. {e}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    logger = MetricsLogger()
    atexit.register(logger.save_data_hook)
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("[LOGGER] KeyboardInterrupt received.", flush=True)
    finally:
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()