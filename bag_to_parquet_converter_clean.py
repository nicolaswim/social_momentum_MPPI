#!/usr/bin/env python3
"""
bag_to_parquet_converter_clean.py

MANUAL RECOVERY V12 (Blind Sweep):
- Odom: Uses confirmed offsets (44, 100).
- JointStates: Ignores headers. Scans the entire binary blob for ANY sequence 
  of 14 consecutive float64s. Picks the sequence that looks like velocity 
  (low values) rather than position (high values).
"""

import os
import sys
import struct
import sqlite3
import pandas as pd
import numpy as np
from pathlib import Path

# --- Configuration ---
ODOM_TOPIC = "/mobile_base_controller/odom"
JS_TOPIC = "/joint_states"

# --- Odom Offsets (Confirmed) ---
ODOM_POS_X = 44
ODOM_POS_Y = 52
ODOM_LIN_VEL = 100

# --- Joint Schema (14 Joints) ---
JOINT_NAMES = [
    "wheel_right_joint", "head_2_joint", "arm_1_joint", "head_1_joint",
    "gripper_right_finger_joint", "arm_2_joint", "arm_3_joint", 
    "gripper_left_finger_joint", "arm_6_joint", "wheel_left_joint", 
    "arm_7_joint", "arm_5_joint", "arm_4_joint", "torso_lift_joint"
]
NUM_JOINTS = 14
# 14 doubles * 8 bytes = 112 bytes of data
ARRAY_BYTE_SIZE = NUM_JOINTS * 8

def get_db_path(bag_dir):
    bag_path = Path(bag_dir)
    db_files = list(bag_path.glob("*.db3"))
    if not db_files: return None
    return db_files[0]

def convert_bag_to_parquet(bag_dir: str, parquet_output: str) -> None:
    db_path = get_db_path(bag_dir)
    if not db_path:
        print(f"[FATAL] No .db3 file found in {bag_dir}")
        return

    print(f"[INFO] Processing: {db_path.name}")
    
    odom_rows = []
    js_rows = []

    try:
        conn = sqlite3.connect(db_path, detect_types=sqlite3.PARSE_DECLTYPES)
        conn.text_factory = bytes
        cursor = conn.cursor()

        cursor.execute("SELECT id, name FROM topics")
        topic_map = {name.decode('utf-8'): id for id, name in cursor.fetchall()}

        # ---------------------------------------------------------
        # 1. ODOMETRY (Confirmed)
        # ---------------------------------------------------------
        if ODOM_TOPIC in topic_map:
            oid = topic_map[ODOM_TOPIC]
            print(f"[INFO] Recovering Odometry...")
            cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {oid}")
            
            for ts, raw in cursor:
                if len(raw) < 108: continue
                try:
                    px = struct.unpack('<d', raw[ODOM_POS_X:ODOM_POS_X+8])[0]
                    py = struct.unpack('<d', raw[ODOM_POS_Y:ODOM_POS_Y+8])[0]
                    lv = struct.unpack('<d', raw[ODOM_LIN_VEL:ODOM_LIN_VEL+8])[0]
                    odom_rows.append({"timestamp": ts / 1e9, "ts_ns": ts, "pos_x": px, "pos_y": py, "lin_vel": lv})
                except: continue
        print(f"  > Extracted {len(odom_rows)} Odom records.")

        # ---------------------------------------------------------
        # 2. JOINT STATES (Blind Sweep)
        # ---------------------------------------------------------
        if JS_TOPIC in topic_map:
            jid = topic_map[JS_TOPIC]
            print(f"[INFO] Recovering JointStates (Scanning for {NUM_JOINTS} doubles)...")
            
            cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {jid}")
            
            for ts, raw in cursor:
                # We ignore headers. We slide a window of 112 bytes (14 doubles)
                # across the message and see if it makes sense.
                
                candidates = []
                
                # Optimization: Data usually starts after byte 100 in these logs
                # Scan every 4 bytes (alignment guess)
                for i in range(0, len(raw) - ARRAY_BYTE_SIZE, 4):
                    try:
                        # Grab 112 bytes
                        chunk = raw[i : i + ARRAY_BYTE_SIZE]
                        floats = struct.unpack(f'<{NUM_JOINTS}d', chunk)
                        
                        # FILTER: Is this valid data?
                        # 1. No NaN/Inf
                        if any(pd.isna(f) or pd.isnull(f) or abs(f) > 1e9 for f in floats):
                            continue
                            
                        # 2. Calc average magnitude
                        avg_mag = sum(abs(f) for f in floats) / NUM_JOINTS
                        
                        # Store candidate with its magnitude
                        candidates.append((avg_mag, floats))
                    except: pass
                
                # DECISION:
                # If we found candidates, we usually have Position (large values) and Velocity (small values).
                # We want Velocity.
                if candidates:
                    # Sort by average magnitude (lowest first)
                    candidates.sort(key=lambda x: x[0])
                    
                    # The sequence with the smallest numbers is almost certainly Velocity
                    best_floats = candidates[0][1]
                    
                    # Create Row
                    row = {"timestamp": ts / 1e9}
                    for name, vel in zip(JOINT_NAMES, best_floats):
                        row[f"{name}_vel"] = vel
                    js_rows.append(row)

        print(f"  > Extracted {len(js_rows)} JointState records.")
        conn.close()

    except Exception as e:
        print(f"[FATAL] {e}")
        return

    # ---------------------------------------------------------
    # 3. MERGE & SAVE
    # ---------------------------------------------------------
    if not odom_rows:
        print("[WARN] No Odom data. Exiting.")
        return

    print("[INFO] Merging dataframes...")
    df_odom = pd.DataFrame(odom_rows).sort_values("timestamp")
    
    if js_rows:
        df_js = pd.DataFrame(js_rows).sort_values("timestamp")
        df_final = pd.merge_asof(df_odom, df_js, on="timestamp", direction="nearest", tolerance=0.15)
    else:
        df_final = df_odom

    # Restore Timestamp
    df_final["timestamp"] = pd.to_datetime(df_final["ts_ns"], unit="ns")
    
    # Clean columns
    final_cols = ["timestamp", "pos_x", "pos_y", "lin_vel"]
    for jn in JOINT_NAMES:
        col = f"{jn}_vel"
        if col in df_final.columns: final_cols.append(col)
            
    df_final = df_final[final_cols]
    
    out_path = Path(parquet_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    df_final.to_parquet(out_path, engine="pyarrow", compression="snappy")
    print(f"[SUCCESS] Saved {len(df_final)} rows to {out_path}")

def _main():
    if len(sys.argv) != 2:
        print("Usage: python3 bag_to_parquet_converter_clean.py <bag_dir>")
        sys.exit(1)
    bag_dir = sys.argv[1]
    out_name = Path(bag_dir).name + ".parquet"
    out_path = Path(bag_dir).parent / out_name
    convert_bag_to_parquet(bag_dir, str(out_path))

if __name__ == "__main__":
    _main()