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
from gazebo_msgs.msg import ModelStates
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MetricsLogger(Node):
    def __init__(self):
        super().__init__("metrics_logger")
        
        # --- Parameters ---
        self.declare_parameter("output_filename", "/home/user/exchange/rosbags/raw/robot_run_data.parquet")
        self.declare_parameter("slop_seconds", 0.5) 

        self.output_filename = self.get_parameter("output_filename").value
        self.slop_seconds = self.get_parameter("slop_seconds").value
        
        # =========================================================
        # SANITY CHECK: CAN WE WRITE TO THE DISK?
        # =========================================================
        print(f"\n[LOGGER] --- DIAGNOSTIC CHECK START ---", flush=True)
        print(f"[LOGGER] Target Directory: {os.path.dirname(self.output_filename)}", flush=True)
        try:
            folder_path = os.path.dirname(self.output_filename)
            os.makedirs(folder_path, exist_ok=True)
            test_file_path = os.path.join(folder_path, "test_write_check.txt")
            with open(test_file_path, "w") as f:
                f.write("SUCCESS! The container has permission to write to this folder.")
            print(f"[LOGGER] SUCCESS: Created test file at: {test_file_path}", flush=True)
        except Exception as e:
            print(f"[LOGGER] !!! FAILURE !!! Could not write to disk. {e}", flush=True)
        print(f"[LOGGER] --- DIAGNOSTIC CHECK END ---\n", flush=True)
        # =========================================================

        self.data_buffer = []
        self.joint_name_order = None
        self.joint_velocity_names = None
        
        # --- Ground Truth Storage ---
        self.human_names = [] 
        self.latest_model_msg = None # Cache for the latest ground truth

        # --- Subscribers ---
        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        print("[LOGGER] Subscribing with BEST_EFFORT reliability...", flush=True)

        # 1. Synchronized Subscribers (Robot Internal Data)
        odom_sub = message_filters.Subscriber(self, Odometry, "/mobile_base_controller/odom", qos_profile=qos_policy)
        joint_sub = message_filters.Subscriber(self, JointState, "/joint_states", qos_profile=qos_policy)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, joint_sub],
            queue_size=50,
            slop=self.slop_seconds,
            allow_headerless=True
        )
        self.ts.registerCallback(self.unified_callback)

        # 2. Independent Subscriber (Ground Truth / No Header)
        # We subscribe normally because ModelStates has no timestamp to sync with.
        self.create_subscription(
            ModelStates,
            "/model_states",
            self.model_states_callback,
            qos_policy
        )

        print("[LOGGER] Node initialized. Waiting for data...", flush=True)

    def model_states_callback(self, msg):
        """
        Runs whenever Gazebo publishes ground truth. 
        We just cache the message and scan for humans.
        """
        self.latest_model_msg = msg
        
        # Auto-Discovery (Run once)
        if not self.human_names:
            found = [name for name in msg.name if any(k in name for k in ['actor', 'human', 'person'])]
            if found:
                self.human_names = sorted(found)
                print(f"[LOGGER] Auto-Discovered Humans: {self.human_names}", flush=True)

    def unified_callback(self, odom_msg, joint_msg):
        """
        Runs when Odom + JointStates are synced.
        We grab the LATEST cached ModelStates here.
        """
        try:
            # 1. Initialize joint names if needed
            if self.joint_name_order is None:
                self.joint_name_order = joint_msg.name
                self.joint_velocity_names = [n + "_vel" for n in self.joint_name_order]
                print(f"[LOGGER] Joints identified: {len(self.joint_name_order)} joints.", flush=True)

            # 2. Extract Standard Robot Data
            timestamp = odom_msg.header.stamp
            pos_x = odom_msg.pose.pose.position.x
            pos_y = odom_msg.pose.pose.position.y
            lin_vel = odom_msg.twist.twist.linear.x

            vel_map = dict(zip(joint_msg.name, joint_msg.velocity))
            joint_velocities = [vel_map.get(name, 0.0) for name in self.joint_name_order]

            # 3. Extract Ground Truth Data (From Cache)
            gt_robot_x, gt_robot_y = 0.0, 0.0
            human_positions = []

            # Initialize human data with 0.0s in case we have names but no current msg
            current_human_data = [0.0, 0.0] * len(self.human_names)

            if self.latest_model_msg is not None:
                model_msg = self.latest_model_msg
                
                # Robot GT
                if "mobile_base" in model_msg.name:
                    r_idx = model_msg.name.index("mobile_base")
                    gt_robot_x = model_msg.pose[r_idx].position.x
                    gt_robot_y = model_msg.pose[r_idx].position.y

                # Humans GT
                temp_human_pos = []
                for h_name in self.human_names:
                    try:
                        h_idx = model_msg.name.index(h_name)
                        temp_human_pos.extend([model_msg.pose[h_idx].position.x, model_msg.pose[h_idx].position.y])
                    except ValueError:
                        temp_human_pos.extend([0.0, 0.0])
                
                if temp_human_pos:
                    current_human_data = temp_human_pos

            # 4. Construct Final Tuple
            data_tuple = (timestamp, pos_x, pos_y, lin_vel) + tuple(joint_velocities) + (gt_robot_x, gt_robot_y) + tuple(current_human_data)
            
            self.data_buffer.append(data_tuple)

            # 5. Auto-save periodically
            if len(self.data_buffer) % 100 == 0:
                self.save_snapshot()

        except Exception as e:
            print(f"[LOGGER] Error in callback: {e}", flush=True)

    def _get_columns(self):
        cols = ["timestamp", "pos_x", "pos_y", "lin_vel"]
        if self.joint_velocity_names:
            cols += self.joint_velocity_names
        cols += ["robot_gt_x", "robot_gt_y"]
        for h_name in self.human_names:
            cols.append(f"{h_name}_x")
            cols.append(f"{h_name}_y")
        return cols

    def save_snapshot(self):
        try:
            columns = self._get_columns()
            df = pd.DataFrame(self.data_buffer, columns=columns)
            df["timestamp"] = df["timestamp"].apply(lambda t: t.sec * 1_000_000_000 + t.nanosec)
            df.to_parquet(self.output_filename, engine="pyarrow", compression="snappy")
            print(f"[LOGGER] . (Saved {len(df)} rows)", end="", flush=True)
        except Exception as e:
            print(f"\n[LOGGER] SAVE FAILED: {e}", flush=True)

    def save_data_hook(self):
        print(f"\n[LOGGER] Shutdown triggered. Buffer size: {len(self.data_buffer)}", flush=True)
        if not self.data_buffer:
            print("[LOGGER] !!! BUFFER EMPTY. NO DATA COLLECTED !!!", flush=True)
            return

        folder_path = os.path.dirname(self.output_filename)
        if folder_path and not os.path.exists(folder_path):
            try:
                os.makedirs(folder_path, exist_ok=True)
            except Exception as e:
                print(f"[LOGGER] Failed to create dir: {e}", flush=True)
                return

        try:
            columns = self._get_columns()
            df = pd.DataFrame(self.data_buffer, columns=columns)
            df["timestamp"] = df["timestamp"].apply(lambda t: t.sec * 1_000_000_000 + t.nanosec)
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