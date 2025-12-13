#!/usr/bin/env python3
"""
bag_scanner.py - Brute-force scan for floating point numbers in raw CDR blobs.
"""
import sqlite3
import struct
import sys
from pathlib import Path

ODOM_TOPIC = "/mobile_base_controller/odom"
JS_TOPIC = "/joint_states"

def scan_floats(data, topic_name):
    print(f"\n=== SCANNING {topic_name} ({len(data)} bytes) ===")
    print(f"{'Offset':<8} | {'Float32 (<f)':<15} | {'Float64 (<d)':<15} | {'BigEnd F32 (>f)':<15}")
    print("-" * 60)
    
    # Scan every byte offset
    for i in range(len(data) - 8):
        # Try Float32 (Little Endian)
        try:
            f32 = struct.unpack('<f', data[i:i+4])[0]
            s_f32 = f"{f32:.4f}" if abs(f32) < 10000 and abs(f32) > 1e-4 else ""
        except: s_f32 = ""

        # Try Float64 (Little Endian)
        try:
            f64 = struct.unpack('<d', data[i:i+8])[0]
            s_f64 = f"{f64:.4f}" if abs(f64) < 10000 and abs(f64) > 1e-4 else ""
        except: s_f64 = ""
        
        # Try Big Endian Float32 (just in case)
        try:
            be_f32 = struct.unpack('>f', data[i:i+4])[0]
            s_be32 = f"{be_f32:.4f}" if abs(be_f32) < 10000 and abs(be_f32) > 1e-4 else ""
        except: s_be32 = ""

        # Only print lines where we found something that looks like a coordinate/velocity
        if s_f32 or s_f64 or s_be32:
            print(f"{i:<8} | {s_f32:<15} | {s_f64:<15} | {s_be32:<15}")

def probe_bag(bag_dir):
    bag_path = Path(bag_dir)
    db_path = bag_path / f"{bag_path.name}_0.db3"
    
    conn = sqlite3.connect(db_path, detect_types=sqlite3.PARSE_DECLTYPES)
    conn.text_factory = bytes
    cursor = conn.cursor()
    
    cursor.execute("SELECT id, name FROM topics")
    topic_map = {name.decode('utf-8'): id for id, name in cursor.fetchall()}

    # Scan Odom
    if ODOM_TOPIC in topic_map:
        oid = topic_map[ODOM_TOPIC]
        cursor.execute(f"SELECT data FROM messages WHERE topic_id = {oid} LIMIT 1")
        data = cursor.fetchone()[0]
        scan_floats(data, ODOM_TOPIC)

    # Scan Joint States
    if JS_TOPIC in topic_map:
        jid = topic_map[JS_TOPIC]
        cursor.execute(f"SELECT data FROM messages WHERE topic_id = {jid} LIMIT 1")
        data = cursor.fetchone()[0]
        scan_floats(data, JS_TOPIC)
        
        # Also print strings for Joint States to check for names
        print("\n--- String Dump (Joint States) ---")
        text_content = ""
        for b in data:
            c = chr(b)
            if c.isprintable(): text_content += c
            else: text_content += "."
        print(text_content)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 bag_scanner.py <bag_dir>")
    else:
        probe_bag(sys.argv[1])