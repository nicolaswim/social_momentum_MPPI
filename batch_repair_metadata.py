#!/usr/bin/env python3
"""
batch_repair_metadata.py

Usage:
    python3 batch_repair_metadata.py <input_root_dir> <output_root_dir>

This script iterates through all subdirectories in <input_root_dir> 
that contain a *.db3 file, copies them, runs 'ros2 bag reindex' to 
reconstruct metadata.yaml, and saves the repaired bags to <output_root_dir>.
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path

def find_bags_to_repair(root_dir: Path) -> list[Path]:
    """Finds all subdirectories containing a *.db3 file."""
    bags_to_repair = []
    
    # Iterate through all immediate subdirectories
    for item in root_dir.iterdir():
        if item.is_dir():
            # Check if a .db3 file exists inside this directory
            if list(item.glob('*.db3')):
                bags_to_repair.append(item)
    return bags_to_repair


def repair_bag(input_bag_dir: Path, output_bag_dir: Path) -> bool:
    """
    Copies a single bag, runs ros2 bag reindex, and saves the result.
    Returns True on success, False on failure.
    """
    print(f"\n[INFO] --- Starting Repair for: {input_bag_dir.name} ---")

    if output_bag_dir.exists():
        print(f"[WARN] Output directory already exists. Deleting: {output_bag_dir}")
        try:
            shutil.rmtree(output_bag_dir)
        except Exception as e:
            print(f"[ERROR] Failed to remove existing directory {output_bag_dir}: {e}")
            return False

    # Copy the original bag to the intended output location
    print(f"[INFO] Copying bag to '{output_bag_dir}'...")
    try:
        shutil.copytree(input_bag_dir, output_bag_dir)
    except Exception as e:
        print(f"[ERROR] Failed to copy bag: {e}")
        return False

    # Step: reindex to reconstruct metadata.yaml
    print(f"[INFO] Running `ros2 bag reindex` on bag: {output_bag_dir}")
    try:
        # Note: This requires ROS 2 to be sourced in the shell!
        subprocess.run(
            ["ros2", "bag", "reindex", str(output_bag_dir)],
            check=True,
            capture_output=False
        )
        print(f"[SUCCESS] Reindex complete. Repaired bag saved to: {output_bag_dir.name}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[CRITICAL ERROR] `ros2 bag reindex` FAILED for {output_bag_dir.name}. Error: {e}")
        # Clean up the failed copy if possible
        shutil.rmtree(output_bag_dir, ignore_errors=True)
        return False


def main() -> None:
    if len(sys.argv) != 3:
        print("Usage: python3 batch_repair_metadata.py <input_root_dir> <output_root_dir>")
        print("\nExample: python3 batch_repair_metadata.py rosbags/raw/NAV2 rosbags/REPAIRED/NAV2")
        sys.exit(1)

    input_root_dir = Path(sys.argv[1]).resolve()
    output_root_dir = Path(sys.argv[2]).resolve()
    
    if not input_root_dir.is_dir():
        print(f"[ERROR] Input root directory does not exist: {input_root_dir}")
        sys.exit(1)

    # Ensure output root directory exists
    output_root_dir.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] Output root directory set to: {output_root_dir}")

    bags_to_repair = find_bags_to_repair(input_root_dir)
    if not bags_to_repair:
        print(f"[WARN] No bag directories (folders containing a *.db3 file) found in: {input_root_dir}")
        sys.exit(0)
    
    print(f"[INFO] Found {len(bags_to_repair)} bags to repair.")
    
    success_count = 0
    fail_count = 0
    
    for input_bag_path in bags_to_repair:
        output_bag_path = output_root_dir / input_bag_path.name
        
        if repair_bag(input_bag_path, output_bag_path):
            success_count += 1
        else:
            fail_count += 1

    print("\n--- Batch Repair Summary ---")
    print(f"Total Bags Found: {len(bags_to_repair)}")
    print(f"✅ Successes: {success_count}")
    print(f"❌ Failures: {fail_count}")
    print("----------------------------")
    if fail_count > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()