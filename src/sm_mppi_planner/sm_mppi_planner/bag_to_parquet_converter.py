#!/usr/bin/env python3
"""
bag_to_parquet_converter.py

Usage:
    python3 bag_to_parquet_converter.py <bag_dir> <output_parquet>

This script:
  1. Copies the original rosbag2 directory to a temporary location.
  2. Runs `ros2 bag reindex` on the copy to reconstruct metadata.yaml.
  3. (Optional) Could run `ros2 bag convert` as a second repair step.
  4. Runs your existing "bag → metrics → Parquet" conversion on the repaired bag.
  5. Cleans up the temporary directory.

This version includes robust error handling to skip messages that cause
rclpy.serialization.RMWError during deserialization.
"""

import os
import sys
import shutil
import tempfile
import subprocess
import traceback
from pathlib import Path

import pandas as pd
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
# Import RMWError explicitly for error handling
from rclpy._rclpy_pybind11 import RMWError 


def repair_bag_if_needed(original_bag_dir: str, use_convert: bool = False) -> str:
    """
    1. Copy the original bag dir into a temp directory
    2. Run `ros2 bag reindex` on the temp copy to reconstruct metadata.yaml
    3. Optionally run `ros2 bag convert` to produce a fully rewritten bag
    4. Return the path to the repaired bag directory
    """
    original_bag_dir = os.path.abspath(original_bag_dir)

    if not os.path.isdir(original_bag_dir):
        raise FileNotFoundError(f"Input bag directory does not exist: {original_bag_dir}")

    # Create a temp root and copy the bag into it
    temp_root = tempfile.mkdtemp(prefix="bag_repair_")
    temp_bag_dir = os.path.join(temp_root, "reindexed_bag")

    print(f"[INFO] Copying bag from '{original_bag_dir}' to '{temp_bag_dir}'...")
    shutil.copytree(original_bag_dir, temp_bag_dir)

    # Step 1–2: reindex to reconstruct metadata.yaml
    print(f"[INFO] Running `ros2 bag reindex` on temp bag: {temp_bag_dir}")
    # Note: Requires ROS 2 environment (Humble) to be sourced in the shell!
    subprocess.run(
        ["ros2", "bag", "reindex", temp_bag_dir],
        check=True
    )
    print("[INFO] Reindex complete.")

    # Optional: extra repair step using `ros2 bag convert`
    if use_convert:
        converted_dir = os.path.join(temp_root, "converted_bag")
        print(f"[INFO] Running `ros2 bag convert` to: {converted_dir}")
        subprocess.run(
            ["ros2", "bag", "convert", "-i", temp_bag_dir, "-o", converted_dir],
            check=True
        )
        print("[INFO] Convert complete.")
        return converted_dir

    # Default: just use the reindexed bag
    return temp_bag_dir


def detect_storage_id(path: str) -> str:
    for entry in os.listdir(path):
        if entry.endswith(".mcap"):
            return "mcap"
        if entry.endswith(".db3"):
            return "sqlite3"
    # Defaulting to sqlite3 if neither is explicitly found
    return "sqlite3"


# Centralized type cache
_type_cache = {}

def get_msg_type(ros_type_name):
    """Dynamically loads and caches the ROS message class from its type name."""
    global _type_cache
    if ros_type_name not in _type_cache:
        _type_cache[ros_type_name] = get_message(ros_type_name)
    return _type_cache[ros_type_name]


def deserialize_safely(topic_name, data, ros_type_name):
    """Safely deserialize raw message data, handling RMWError."""
    try:
        msg_type = get_msg_type(ros_type_name)
        return deserialize_message(data, msg_type) 
    except RMWError as e:
        # This catches the 'invalid data size' error and skips the corrupted message.
        print(f"[ERROR] RMW Deserialization failed for topic '{topic_name}' (Type: {ros_type_name}). Skipping message. Error: {e}")
        # print(traceback.format_exc()) # Uncomment for deep debugging
        return None 
    except Exception as e:
        # Catch other unexpected errors during deserialization
        print(f"[ERROR] Unexpected error during deserialization for topic '{topic_name}'. Skipping message. Error: {e}")
        return None


def convert_bag_to_parquet(bag_dir: str, parquet_output: str) -> None:
    """
    Reads the repaired bag, extracts Odometry and JointState metrics, 
    synchronizes them, and writes the resulting DataFrame to a Parquet file.
    """
    print(f"[DEBUG] Converting bag '{bag_dir}' to parquet '{parquet_output}'...")

    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f"Repaired bag directory not found: {bag_dir}")

    storage_id = detect_storage_id(bag_dir)
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=os.path.abspath(bag_dir), storage_id=storage_id)
    converter_opts = ConverterOptions("", "")
    reader.open(storage_opts, converter_opts)

    topics_meta = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics_meta}

    odom_topic = "/mobile_base_controller/odom"
    joints_topic = "/joint_states"
    
    # Check for required topics
    if odom_topic not in topic_types:
        print(f"[WARN] Odometry topic '{odom_topic}' not found in bag.")
    if joints_topic not in topic_types:
        print(f"[WARN] Joint States topic '{joints_topic}' not found in bag.")

    if odom_topic not in topic_types and joints_topic not in topic_types:
         raise RuntimeError("Bag does not contain required topics '/mobile_base_controller/odom' AND '/joint_states'. Conversion aborted.")


    odom_rows = []
    joint_rows = []
    joint_name_order = None

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        
        # Only process the topics we care about
        if topic_name not in topic_types or topic_name not in (odom_topic, joints_topic):
            continue

        ros_type_name = topic_types[topic_name]
        msg = deserialize_safely(topic_name, data, ros_type_name)

        if msg is None:
            continue # Skip corrupted message

        if topic_name == odom_topic:
            # Check if topic type matches expected Odometry message structure (basic check)
            if not hasattr(msg, 'pose') or not hasattr(msg, 'twist'):
                print(f"[WARN] Skipping message on {odom_topic}: Message structure unexpected.")
                continue

            row = {
                "timestamp_ns": int(timestamp),
                "pos_x": msg.pose.pose.position.x,
                "pos_y": msg.pose.pose.position.y,
                "lin_vel": msg.twist.twist.linear.x,
            }
            odom_rows.append(row)
            
        elif topic_name == joints_topic:
            # Check if topic type matches expected JointState message structure
            if not hasattr(msg, 'name') or not hasattr(msg, 'velocity'):
                print(f"[WARN] Skipping message on {joints_topic}: Message structure unexpected.")
                continue

            if joint_name_order is None:
                joint_name_order = list(msg.name)
            
            # Map velocities to joint names
            vel_map = dict(zip(msg.name, msg.velocity))
            row = {"timestamp_ns": int(timestamp)}
            
            # Use the established joint order for consistent columns
            for name in joint_name_order:
                row[f"{name}_vel"] = vel_map.get(name, float("nan"))
            joint_rows.append(row)

    if not odom_rows and odom_topic in topic_types:
        print(f"[WARN] No valid Odometry data found after filtering.")
    if not joint_rows and joints_topic in topic_types:
        print(f"[WARN] No valid Joint State data found after filtering.")

    if not odom_rows or not joint_rows or joint_name_order is None:
        raise RuntimeError("Not enough valid Odometry or Joint State data to perform synchronization and write Parquet.")

    # --- DataFrame Processing ---
    odom_df = pd.DataFrame(odom_rows).sort_values("timestamp_ns")
    joint_df = pd.DataFrame(joint_rows).sort_values("timestamp_ns")

    # Time-series merge (synchronization)
    tolerance_ns = int(0.15 * 1e9) # 150 milliseconds tolerance
    merged = pd.merge_asof(
        odom_df,
        joint_df,
        on="timestamp_ns",
        direction="nearest",
        tolerance=tolerance_ns,
    )
    
    # Drop rows where synchronization failed (i.e., missing joint velocities)
    joint_cols = [f"{name}_vel" for name in joint_name_order]
    merged = merged.dropna(subset=joint_cols)

    if merged.empty:
        raise RuntimeError(f"Failed to synchronize odometry and joint state samples within tolerance ({tolerance_ns} ns). Output DataFrame is empty.")

    # Final formatting
    merged["timestamp"] = pd.to_datetime(merged["timestamp_ns"], unit="ns")
    merged = merged.drop(columns=["timestamp_ns"])
    ordered_columns = ["timestamp", "pos_x", "pos_y", "lin_vel"] + joint_cols
    merged = merged[ordered_columns]

    # Ensure output directory exists
    out_dir = os.path.dirname(os.path.abspath(parquet_output))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    # Write to Parquet
    merged.to_parquet(parquet_output, index=False, engine="pyarrow", compression="snappy")
    print(f"[INFO] Wrote {len(merged)} synchronized samples to {parquet_output}")


def main() -> None:
    if len(sys.argv) != 3:
        print("Usage: python3 bag_to_parquet_converter.py <bag_dir> <output_parquet>")
        sys.exit(1)

    input_bag_dir = sys.argv[1]
    output_parquet = sys.argv[2]

    # Step 1–3: repair the bag automatically (copy + reindex [+ optional convert])
    try:
        repaired_bag_dir = repair_bag_if_needed(input_bag_dir, use_convert=True)
    except Exception as e:
        print(f"[ERROR] Failed to repair bag: {e}")
        sys.exit(1)

    # Step 4: run your existing bag→parquet logic on the repaired bag
    temp_root = Path(repaired_bag_dir).parent

    try:
        convert_bag_to_parquet(str(repaired_bag_dir), output_parquet)
    finally:
        # Step 5: clean up the temporary directories
        print(f"[INFO] Cleaning up temp directory: {temp_root}")
        shutil.rmtree(temp_root, ignore_errors=True)


if __name__ == "__main__":
    main()