#!/usr/bin/env python3
"""
batch_bag_to_parquet.py

Usage:
    python3 batch_bag_to_parquet.py <input_repaired_root_dir> <output_parquet_root_dir>

This script:

  - Scans <input_repaired_root_dir> for subdirectories containing a metadata.yaml
    (i.e. repaired rosbag2 directories).
  - For each such directory, calls convert_bag_to_parquet(...) from
    bag_to_parquet_converter_clean.py, which:
       * safely deserialises messages (skipping corrupted ones)
       * extracts odom + joint_states
       * writes a single .parquet file.
  - Stores all resulting .parquet files in <output_parquet_root_dir>.
"""

import os
import sys
from pathlib import Path

# Import the main conversion function from your cleaner script
from bag_to_parquet_converter_clean import convert_bag_to_parquet as convert_single_bag_to_parquet


def find_repaired_bags(root_dir: Path) -> list[Path]:
    """Find all subdirectories that contain a metadata.yaml file."""
    bags_to_convert = []

    for item in root_dir.iterdir():
        if item.is_dir() and (item / "metadata.yaml").exists():
            bags_to_convert.append(item)

    return bags_to_convert


def batch_convert(input_root: str, output_root: str) -> None:
    """Handles the batch processing and conversion."""
    input_root_dir = Path(input_root).resolve()
    output_root_dir = Path(output_root).resolve()

    if not input_root_dir.is_dir():
        print(f"[ERROR] Input directory not found: {input_root_dir}")
        sys.exit(1)

    output_root_dir.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] Parquet output directory set to: {output_root_dir}")

    bags_to_convert = find_repaired_bags(input_root_dir)
    if not bags_to_convert:
        print(f"[WARN] No repaired bag directories with metadata.yaml found in: {input_root_dir}")
        sys.exit(0)

    print(f"[INFO] Found {len(bags_to_convert)} repaired bags to convert.")

    success_count = 0
    fail_count = 0

    for input_bag_path in bags_to_convert:
        output_filename = f"{input_bag_path.name}.parquet"
        output_parquet_path = output_root_dir / output_filename

        print(f"\n[INFO] --- Converting Bag: {input_bag_path.name} ---")
        print(f"[INFO] Input:  {input_bag_path}")
        print(f"[INFO] Output: {output_parquet_path}")

        try:
            convert_single_bag_to_parquet(str(input_bag_path), str(output_parquet_path))
            success_count += 1
        except Exception as e:
            print(f"[CRITICAL ERROR] Conversion FAILED for {input_bag_path.name}: {e}")
            fail_count += 1

    print("\n--- Batch Conversion Summary ---")
    print(f"Total Bags Attempted: {len(bags_to_convert)}")
    print(f"✅ Successes: {success_count}")
    print(f"❌ Failures: {fail_count}")
    print("------------------------------")
    if fail_count > 0:
        sys.exit(1)


def main() -> None:
    if len(sys.argv) != 3:
        print("Usage: python3 batch_bag_to_parquet.py <input_repaired_root_dir> <output_parquet_root_dir>")
        sys.exit(1)

    input_root = sys.argv[1]
    output_root = sys.argv[2]

    batch_convert(input_root, output_root)


if __name__ == "__main__":
    main()
