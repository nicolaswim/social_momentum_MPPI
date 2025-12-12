#!/usr/bin/env python3

## Component 2: Social Navigation Analysis Dashboard
##
## Adaptations:
## - COMPATIBILITY FIX: Added .values to df.index and all columns.
## - RESTORED: --folder argument support.
## - Fixed Map Frame (Hallway view).
## - "Social Compliance" Scatter Plot.
## - "Safety Monitor" (Distance to nearest human).
## - Single-score "Smoothness" metric.
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import argparse

# --- PHYSICS LIMITS ---
MAX_POS_RANGE = 5000.0 # Discard points > 5km (Space Junk)

# --- HALLWAY DIMENSIONS (Fixed Frame) ---
HALL_X_MIN, HALL_X_MAX = -1.0, 12.0
HALL_Y_MIN, HALL_Y_MAX = -3.0, 3.0  # 6m wide view for a 5m hall

def load_data(parquet_file):
    if not os.path.exists(parquet_file): return None
    print(f"Loading {parquet_file}...")
    try:
        df = pd.read_parquet(parquet_file, engine="pyarrow")
    except Exception as e:
        print(f"[ERROR] Corrupt file: {e}")
        return None
    
    # Time column normalization
    time_col = "timestamp"
    if time_col not in df.columns:
        cols = [c for c in df.columns if "time" in c.lower()]
        time_col = cols[0] if cols else None
    
    if not time_col: return None

    # De-duplicate and Sort
    df = df.drop_duplicates()
    if pd.api.types.is_numeric_dtype(df[time_col]):
        df[time_col] = pd.to_datetime(df[time_col], unit='ns')

    df.set_index(time_col, inplace=True)
    df.sort_index(inplace=True)
    
    # SANITIZE: Remove space junk
    if "pos_x" in df.columns:
        df = df[df["pos_x"].abs() < MAX_POS_RANGE]

    return df

def calculate_social_metrics(df):
    """
    Calculates advanced social navigation metrics if data is available.
    """
    stats = {}
    
    # 1. Standard Kinematics (for Smoothness Score)
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds()
    df = df[df["delta_t"] > 0.0001].copy() # Filter zero-time steps
    
    df["accel"] = df["lin_vel"].diff() / df["delta_t"]
    df["jerk"] = df["accel"].diff() / df["delta_t"]
    
    # SMOOTHNESS SCORE: Average Absolute Jerk (Lower is better)
    stats["smoothness_score"] = df["jerk"].abs().rolling(10).mean().mean()

    # 2. Path Efficiency (SPL approx)
    dist = np.sqrt(df["pos_x"].diff()**2 + df["pos_y"].diff()**2)
    stats["path_length"] = dist.sum()
    
    # 3. Social Metrics (Distance to Humans)
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    human_prefixes = [c.replace("_x", "") for c in human_x_cols]
    
    if human_prefixes:
        dist_cols = []
        for h in human_prefixes:
            # Use Ground Truth robot position if available, else Odom
            rx = df["gt_robot_x"] if "gt_robot_x" in df.columns else df["pos_x"]
            ry = df["gt_robot_y"] if "gt_robot_y" in df.columns else df["pos_y"]
            
            hx = df[f"{h}_x"]
            hy = df[f"{h}_y"]
            
            # Vectorized Distance
            d_col = f"dist_to_{h}"
            df[d_col] = np.sqrt((rx - hx)**2 + (ry - hy)**2)
            dist_cols.append(d_col)
        
        # Find the MIN distance to ANY human at each timestamp
        df["nearest_human_dist"] = df[dist_cols].min(axis=1)
        
        stats["min_human_dist"] = df["nearest_human_dist"].min()
        stats["avg_human_dist"] = df["nearest_human_dist"].mean()
        stats["has_social_data"] = True
    else:
        stats["min_human_dist"] = -1
        stats["has_social_data"] = False

    stats["duration"] = (df.index.max() - df.index.min()).total_seconds()
    return df, stats

def create_dashboard(df, stats, filename, output_dir):
    print("Generating dashboard...")
    
    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(f"Social Nav Analysis: {os.path.basename(filename)}", fontsize=16)

    # ==========================================================
    # 1. TOP LEFT: The Hallway Map (Context)
    # ==========================================================
    ax1 = axs[0, 0]
    ax1.set_title("Hallway Activity Map (Fixed Frame)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    
    # Wall Lines
    ax1.axhline(y=2.5, color='k', linewidth=3)
    ax1.axhline(y=-2.5, color='k', linewidth=3)
    ax1.set_xlim(HALL_X_MIN, HALL_X_MAX)
    ax1.set_ylim(HALL_Y_MIN, HALL_Y_MAX)
    
    # Human Trajectories
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    for h_col in human_x_cols:
        h_prefix = h_col.replace("_x", "")
        valid_mask = df[h_col].abs() < 50
        # FIX: Added .values here
        ax1.plot(df.loc[valid_mask, f"{h_prefix}_x"].values, 
                 df.loc[valid_mask, f"{h_prefix}_y"].values, 
                 color="grey", alpha=0.3, linewidth=1)

    # Robot Path
    if "gt_robot_x" in df.columns:
        # FIX: Added .values here
        ax1.plot(df["gt_robot_x"].values, df["gt_robot_y"].values, "g--", label="Ground Truth", alpha=0.7)
    
    # FIX: Added .values here
    ax1.plot(df["pos_x"].values, df["pos_y"].values, "b-", label="Odom", linewidth=2)
    
    # Start/End (iloc returns scalar, so no .values needed)
    ax1.plot(df["pos_x"].iloc[0], df["pos_y"].iloc[0], "go", label="Start")
    ax1.plot(df["pos_x"].iloc[-1], df["pos_y"].iloc[-1], "rx", label="End")
    ax1.legend(loc="upper right")
    ax1.grid(True, linestyle=':')

    # ==========================================================
    # 2. TOP RIGHT: Safety Monitor (Proxemics)
    # ==========================================================
    ax2 = axs[0, 1]
    if stats["has_social_data"]:
        ax2.set_title("Safety Monitor: Proximity to Humans")
        ax2.set_ylabel("Distance (m)")
        ax2.set_xlabel("Time")
        
        # FIX: Added .values to df.index as well!
        ax2.plot(df.index.values, df["nearest_human_dist"].values, color="orange", label="Dist to Nearest")
        
        # Danger Lines
        ax2.axhline(y=0.45, color='r', linestyle='--', linewidth=2, label="Intimate Zone (0.45m)")
        ax2.axhline(y=1.2, color='y', linestyle=':', label="Personal Zone (1.2m)")
        
        failures = df[df["nearest_human_dist"] < 0.45]
        if not failures.empty:
            # FIX: Added .values to failures.index as well!
            ax2.plot(failures.index.values, failures["nearest_human_dist"].values, "rx", label="Intrusion")

        ax2.legend(loc="upper right")
    else:
        ax2.text(0.5, 0.5, "NO HUMAN DATA DETECTED", ha='center')

    ax2.grid(True)

    # ==========================================================
    # 3. BOTTOM LEFT: Social Compliance (Scatter)
    # ==========================================================
    ax3 = axs[1, 0]
    if stats["has_social_data"]:
        ax3.set_title("Politeness Check: Velocity vs. Proximity")
        ax3.set_xlabel("Distance to Nearest Human (m)")
        ax3.set_ylabel("Robot Velocity (m/s)")
        
        # FIX: Added .values here
        sc = ax3.scatter(df["nearest_human_dist"].values, df["lin_vel"].values, 
                         c=np.arange(len(df)), cmap="Blues", alpha=0.5, s=10)
        ax3.grid(True)
    else:
        ax3.text(0.5, 0.5, "NO HUMAN DATA DETECTED", ha='center')

    # ==========================================================
    # 4. BOTTOM RIGHT: The Report Card
    # ==========================================================
    ax4 = axs[1, 1]
    ax4.axis("off")
    
    lines = [
        "--- RUN REPORT CARD ---",
        f"Duration:     {stats['duration']:.2f} s",
        f"Path Length:  {stats['path_length']:.2f} m",
        "",
        "--- COMFORT METRICS ---",
        f"Smoothness:   {stats['smoothness_score']:.1f} (Lower is better)",
        "(Avg Abs Jerk)",
        "",
        "--- SAFETY METRICS ---",
    ]
    
    if stats["has_social_data"]:
        min_d = stats['min_human_dist']
        status = "FAIL (Collision)" if min_d < 0.2 else \
                 "FAIL (Intrusion)" if min_d < 0.45 else "PASS"
        
        lines.append(f"Min Dist:     {min_d:.2f} m")
        lines.append(f"Safety Check: {status}")
    else:
        lines.append("No Human Data Available")

    ax4.text(0.1, 0.9, "\n".join(lines), transform=ax4.transAxes, 
             fontsize=12, family="monospace", va="top")

    # Save
    plt.tight_layout()
    img_path = os.path.join(output_dir, os.path.splitext(os.path.basename(filename))[0] + ".png")
    plt.savefig(img_path)
    print(f"Saved: {img_path}")
    plt.close(fig)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root_dir", help="Path to raw data (e.g. rosbags/raw)")
    parser.add_argument("--folder", help="Specific subfolder", default=None)
    args = parser.parse_args()
    
    raw_root = os.path.abspath(args.root_dir)
    # Adjust post_processing location
    if os.path.basename(raw_root) == "raw":
        base_dir = os.path.dirname(raw_root)
        post_root = os.path.join(base_dir, "post_processing")
    else:
        post_root = os.path.join(raw_root, "post_processing")

    # Filter Folders
    if args.folder:
        target = os.path.join(raw_root, args.folder)
        if not os.path.isdir(target):
            print(f"Error: Folder '{args.folder}' not found in {raw_root}")
            sys.exit(1)
        subdirs = [target]
    else:
        subdirs = [os.path.join(raw_root, d) for d in os.listdir(raw_root) if os.path.isdir(os.path.join(raw_root, d))]
    
    print(f"Processing {len(subdirs)} folders...")

    for source in subdirs:
        folder_name = os.path.basename(source)
        out_dir = os.path.join(post_root, folder_name)
        os.makedirs(out_dir, exist_ok=True)
        
        print(f"\nProcessing {folder_name}...")
        files = [f for f in sorted(os.listdir(source)) if f.endswith(".parquet")]
        
        if not files:
            print("  No parquet files found.")
            continue

        for f in files:
            df = load_data(os.path.join(source, f))
            if df is not None and not df.empty:
                df, stats = calculate_social_metrics(df)
                create_dashboard(df, stats, f, out_dir)

if __name__ == "__main__":
    main()