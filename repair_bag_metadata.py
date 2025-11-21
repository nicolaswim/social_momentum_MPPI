#!/usr/bin/env python3
"""
repair_bag_metadata.py

Usage:
    python3 repair_bag_metadata.py <input_bag_dir> <output_bag_dir>

This script copies an input ROS 2 bag (which may be missing metadata.yaml) 
and runs `ros2 bag reindex` on the copy to reconstruct the metadata.yaml.
The successfully repaired bag is saved permanently to <output_bag_dir>.
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path

def repair_bag_and_save(input_bag_dir: str, output_bag_dir: str) -> None:
    """
    Copies the input bag, runs ros2 bag reindex, and saves the result 
    to the final output directory.
    """
    input_bag_dir = Path(input_bag_dir).resolve()
    output_bag_dir = Path(output_bag_dir).resolve()

    if not input_bag_dir.is_dir():
        raise FileNotFoundError(f"Input bag directory does not exist: {input_bag_dir}")

    if output_bag_dir.exists():
        print(f"[WARN] Output directory already exists. Deleting: {output_bag_dir}")
        shutil.rmtree(output_bag_dir)
        
    # Copy the original bag directly to the intended output location
    print(f"[INFO] Copying bag from '{input_bag_dir}' to '{output_bag_dir}'...")
    shutil.copytree(input_bag_dir, output_bag_dir)
    
    # Step: reindex to reconstruct metadata.yaml
    print(f"[INFO] Running `ros2 bag reindex` on bag: {output_bag_dir}")
    try:
        # Note: This requires ROS 2 to be sourced in the shell where this script is run!
        subprocess.run(
            ["ros2", "bag", "reindex", str(output_bag_dir)],
            check=True,
            capture_output=False  # Show output for debugging
        )
        print(f"[INFO] Reindex complete. Repaired bag saved to: {output_bag_dir}")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] `ros2 bag reindex` failed! The bag may be severely corrupted.")
        raise RuntimeError(f"Reindex failed with exit code {e.returncode}. Output:\n{e.stderr.decode()}")


def main() -> None:
    if len(sys.argv) != 3:
        print("Usage: python3 repair_bag_metadata.py <input_bag_dir> <output_bag_dir>")
        sys.exit(1)

    input_bag_dir = sys.argv[1]
    output_bag_dir = sys.argv[2]

    try:
        repair_bag_and_save(input_bag_dir, output_bag_dir)
    except Exception as e:
        print(f"[CRITICAL ERROR] Failed to complete metadata repair: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()