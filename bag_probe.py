#!/usr/bin/env python3
"""
bag_probe.py - Diagnostic tool to inspect raw ROS 2 CDR byte alignment.
"""

import sqlite3
import struct
import sys
import yaml
import os
from pathlib import Path

ODOM_TOPIC = "/mobile_base_controller/odom"
JS_TOPIC = "/joint_states"

def get_db_path(bag_dir):
    bag_path = Path(bag_dir)
    return bag_path / f"{bag_path.name}_0.db3"

def read_string(data, cursor):
    try:
        str_len = struct.unpack('<I', data[cursor:cursor+4])[0]
        cursor += 4
        s = data[cursor:cursor+str_len].decode('utf-8', errors='replace')
        cursor += str_len
        cursor += (4 - (str_len % 4)) % 4 # Padding
        return s, cursor
    except Exception as e:
        return f"<ERR:{e}>", cursor

def probe_bag(bag_dir):
    db_path = get_db_path(bag_dir)
    if not db_path.exists():
        print(f"DB not found: {db_path}")
        return

    # We want to handle TEXT columns as bytes to avoid decoding errors, 
    # but we must pass SQL queries as strings.
    conn = sqlite3.connect(db_path, detect_types=sqlite3.PARSE_DECLTYPES)
    conn.text_factory = bytes 
    cursor = conn.cursor()

    # FIX: SQL query is a standard string
    cursor.execute("SELECT id, name FROM topics")
    topic_map = {name.decode('utf-8'): id for id, name in cursor.fetchall()}

    # --- PROBE ODOMETRY ---
    if ODOM_TOPIC in topic_map:
        print(f"\n=== PROBING ODOMETRY ({ODOM_TOPIC}) ===")
        oid = topic_map[ODOM_TOPIC]
        
        # FIX: SQL query is a standard string
        cursor.execute(f"SELECT data FROM messages WHERE topic_id = {oid} LIMIT 1")
        row = cursor.fetchone()
        
        if row:
            data = row[0]
            print(f"Total Msg Length: {len(data)} bytes")
            
            # 1. Analyze Header
            cursor_idx = 4 # CDR
            cursor_idx += 8 # Time
            frame_id, cursor_idx = read_string(data, cursor_idx)
            print(f"Header read: frame_id='{frame_id}', cursor_after_header={cursor_idx}")
            
            child_frame_id, cursor_idx = read_string(data, cursor_idx)
            print(f"Child Frame ID read: '{child_frame_id}', cursor_after_child={cursor_idx}")
            
            # 2. Test Pose Offsets around the current cursor
            # We expect Pose to start at an 8-byte aligned position near the cursor
            print("\n--- Hunting for Pose.Position.x (float64) ---")
            start_search = cursor_idx - 8
            end_search = cursor_idx + 16
            
            for i in range(start_search, end_search):
                # Align check (allow checking 4-byte alignments too just in case)
                if i % 4 != 0: continue 
                
                try:
                    val = struct.unpack('<d', data[i:i+8])[0]
                    # Heuristic: Position should be small (e.g. 0.0 to 100.0), not 1e-300
                    valid_mark = " [LIKELY]" if -1000.0 < val < 1000.0 and abs(val) > 1e-9 else ""
                    print(f"Offset {i}: {val:.5f} {valid_mark}")
                except:
                    pass

            # 3. Test Twist Offset (Linear Velocity)
            # Assuming standard Odom: Pose(7 dbl) + Cov(36 dbl) = 43 doubles = 344 bytes
            # Twist starts 344 bytes after Pose start
            
            # Let's assume the 'cursor_after_child' + alignment is the Pose Start
            aligned_pose_start = cursor_idx + ((8 - (cursor_idx % 8)) % 8)
            twist_start_guess = aligned_pose_start + 344
            
            print(f"\n--- Hunting for Twist.Linear.x (at ~{twist_start_guess}?) ---")
            for i in range(twist_start_guess - 8, twist_start_guess + 8):
                try:
                    val = struct.unpack('<d', data[i:i+8])[0]
                     # Heuristic: Velocity is usually small (< 5.0 m/s)
                    valid_mark = " [LIKELY VEL]" if -10.0 < val < 10.0 else ""
                    print(f"Offset {i}: {val:.5f} {valid_mark}")
                except:
                    pass

    # --- PROBE JOINT STATES ---
    if JS_TOPIC in topic_map:
        print(f"\n=== PROBING JOINT STATES ({JS_TOPIC}) ===")
        jid = topic_map[JS_TOPIC]
        
        # FIX: SQL query is a standard string
        cursor.execute(f"SELECT data FROM messages WHERE topic_id = {jid} LIMIT 1")
        row = cursor.fetchone()
        
        if row:
            data = row[0]
            print(f"Total Msg Length: {len(data)} bytes")
            
            # 1. Header
            cursor_idx = 4 + 8
            fid, cursor_idx = read_string(data, cursor_idx)
            print(f"Header read: frame_id='{fid}', cursor={cursor_idx}")
            
            # 2. Name List
            try:
                list_len = struct.unpack('<I', data[cursor_idx:cursor_idx+4])[0]
                print(f"Name List Length: {list_len}")
                cursor_idx += 4
                
                names = []
                # Just read 5 names to confirm
                for _ in range(min(list_len, 5)): 
                    s, cursor_idx = read_string(data, cursor_idx)
                    names.append(s)
                print(f"First few names: {names}")
                
                # RE-CALCULATE exact cursor after all names
                cursor_idx = 4 + 8 # Reset
                _, cursor_idx = read_string(data, cursor_idx) # Header
                list_len = struct.unpack('<I', data[cursor_idx:cursor_idx+4])[0]
                cursor_idx += 4
                for _ in range(list_len):
                    _, cursor_idx = read_string(data, cursor_idx)
                
                print(f"Cursor after Names: {cursor_idx}")
                
                # 3. Position List
                # Align to 8 bytes
                align_pad = (8 - (cursor_idx % 8)) % 8
                cursor_idx += align_pad
                print(f"Aligned Cursor for Positions: {cursor_idx}")
                
                pos_len = struct.unpack('<I', data[cursor_idx:cursor_idx+4])[0]
                print(f"Position List Length: {pos_len}")
                cursor_idx += 4
                
                # 4. Velocity List Start (approx)
                vel_start_est = cursor_idx + (pos_len * 8)
                print(f"Estimated Velocity Start: {vel_start_est}")

            except Exception as e:
                print(f"JointState parsing failed: {e}")

    conn.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 bag_probe.py <bag_dir>")
    else:
        probe_bag(sys.argv[1])