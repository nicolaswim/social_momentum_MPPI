#!/usr/bin/env python3

## Component 2: Offline Analysis and Visualization
##
## Repairs:
## - Handles duplicate timestamps (division by zero error).
## - Cleans Inf/NaN values before plotting (Matplotlib crash fix).
## - Allows filtering by specific subfolder using --folder argument.
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import argparse

def load_data(parquet_file):
    if not os.path.exists(parquet_file):
        print(f"File not found: {parquet_file}")
        sys.exit(1)
        
    print(f"Loading data from {parquet_file}...")
    df = pd.read_parquet(parquet_file, engine="pyarrow")
    
    ACTUAL_TIME_COLUMN = "timestamp"
    
    if ACTUAL_TIME_COLUMN not in df.columns:
        # Try to find a timestamp-like column
        cols = [c for c in df.columns if "time" in c.lower()]
        if cols:
            ACTUAL_TIME_COLUMN = cols[0]
            print(f"[WARN] Default timestamp not found. Using '{ACTUAL_TIME_COLUMN}'")
        else:
            raise KeyError(f"Column '{ACTUAL_TIME_COLUMN}' not found. Columns: {list(df.columns)}")
        
    df = df.drop_duplicates()
    df.set_index(ACTUAL_TIME_COLUMN, inplace=True)
    df.sort_index(inplace=True)
    
    # CRITICAL FIX: Remove duplicate index times
    df = df[~df.index.duplicated(keep='first')]
    
    print(f"Loaded {len(df)} unique data points.")
    return df

def calculate_metrics(df):
    print("Calculating metrics...")
    
    # Time diff in seconds
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds()
    
    # CRITICAL FIX: Filter bad time deltas
    df = df[df["delta_t"] > 0.000001].copy()

    # Derivatives
    df["acceleration_m_s2"] = df["lin_vel"].diff() / df["delta_t"]
    df["jerk_m_s3"] = df["acceleration_m_s2"].diff() / df["delta_t"]

    # CRITICAL FIX: Clean Infinities
    df.replace([np.inf, -np.inf], np.nan, inplace=True)
    df.dropna(subset=["acceleration_m_s2", "jerk_m_s3", "pos_x", "pos_y"], inplace=True)

    if df.empty:
        return df, {"total_duration": 0, "avg_joint_speeds": {}}

    # KPIs
    total_duration = df.index.max() - df.index.min()
    joint_vel_cols = [col for col in df.columns if col.endswith("_vel") and col != "lin_vel"]
    
    if joint_vel_cols:
        avg_joint_speeds = df[joint_vel_cols].abs().mean()
    else:
        avg_joint_speeds = {}
    
    kpi_summary = {
        "total_duration": total_duration,
        "avg_joint_speeds": avg_joint_speeds,
    }
    
    return df, kpi_summary

def create_dashboard(df, kpi_summary, parquet_file, output_dir, show_plot=True):
    if df.empty:
        print("Skipping dashboard (Empty Data)")
        return

    print("Generating dashboard...")
    fig, axs = plt.subplots(2, 2, figsize=(18, 12))
    fig.suptitle(f"Robot Performance: {os.path.basename(parquet_file)}", fontsize=16)

    # Plot 1: Path
    ax1 = axs[0, 0]
    ax1.plot(df["pos_x"].values, df["pos_y"].values, color="blue", label="Path", linewidth=1)
    if len(df) > 0:
        ax1.plot(df["pos_x"].iloc[0], df["pos_y"].iloc[0], "go", markersize=8, label="Start")
        ax1.plot(df["pos_x"].iloc[-1], df["pos_y"].iloc[-1], "rs", markersize=8, label="End")
    ax1.set_title("Robot Path (Odom)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend()
    ax1.grid(True)
    ax1.axis("equal")

    # Plot 2: Kinematics
    ax2 = axs[0, 1]
    ax2.set_title("Kinematics")
    ax2.set_xlabel("Sample")
    ax2.set_ylabel("Accel ($m/s^2$)", color="green")
    
    x_axis = np.arange(len(df))
    l1 = ax2.plot(x_axis, df["acceleration_m_s2"].values, color="green", alpha=0.7, label="Accel")
    
    ax2_twin = ax2.twinx()
    ax2_twin.set_ylabel("Jerk ($m/s^3$)", color="purple")
    l2 = ax2_twin.plot(x_axis, df["jerk_m_s3"].values, color="purple", alpha=0.5, label="Jerk")
    
    ax2.legend(l1+l2, [l.get_label() for l in l1+l2], loc="upper right")
    ax2.grid(True)

    # Plot 3: Joints
    ax3 = axs[1, 0]
    joint_cols = [c for c in df.columns if c.endswith("_vel") and c != "lin_vel"]
    if joint_cols:
        top_joints = df[joint_cols].abs().sum().nlargest(5).index.tolist()
        for col in top_joints:
            ax3.plot(x_axis, df[col].values, label=col.replace("_joint_vel",""), alpha=0.8)
        ax3.legend(loc="upper right", fontsize='small')
    ax3.set_title("Top 5 Active Joints")
    ax3.set_xlabel("Sample")
    ax3.set_ylabel("Rad/s")
    ax3.grid(True)

    # Plot 4: Text
    ax4 = axs[1, 1]
    ax4.axis("off")
    duration_str = str(kpi_summary["total_duration"]).split(".")[0]
    text_lines = [
        "--- STATISTICS ---",
        f"Duration: {duration_str}",
        f"Samples:  {len(df)}",
        "",
        "--- AVG JOINT VEL (rad/s) ---"
    ]
    if isinstance(kpi_summary["avg_joint_speeds"], pd.Series):
        for joint, speed in kpi_summary["avg_joint_speeds"].items():
            text_lines.append(f"{joint.replace('_joint_vel',''):<20}: {speed:.3f}")

    ax4.text(0.05, 0.95, "\n".join(text_lines), transform=ax4.transAxes, 
             fontsize=10, verticalalignment="top", fontfamily="monospace")

    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    file_name = os.path.splitext(os.path.basename(parquet_file))[0]
    output_image = os.path.join(output_dir, file_name + ".png")
    
    plt.savefig(output_image)
    print(f"Dashboard saved: {output_image}")
    plt.close(fig)

def process_file(parquet_file, output_dir, show_plot):
    try:
        df = load_data(parquet_file)
        df_metrics, kpi_summary = calculate_metrics(df)
        create_dashboard(df_metrics, kpi_summary, parquet_file, output_dir, show_plot)
    except Exception as e:
        print(f"[ERROR] Failed to process {parquet_file}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Process Parquet files into Dashboards.")
    parser.add_argument("root_dir", help="Path to the raw data directory (e.g., rosbags/raw)")
    parser.add_argument("--folder", help="Specific subfolder to process (e.g., REPAIRED_NAV2)", default=None)
    
    args = parser.parse_args()
    
    raw_root = os.path.abspath(args.root_dir)
    
    if not os.path.isdir(raw_root):
        print(f"Error: {raw_root} is not a directory.")
        sys.exit(1)

    # Determine post_processing root (same level as raw)
    # Assumption: raw_root is .../rosbags/raw
    # Post_processing is .../rosbags/post_processing
    rosbags_dir = os.path.dirname(raw_root)
    post_processing_root = os.path.join(rosbags_dir, "post_processing")

    # Find folders to process
    if args.folder:
        target_folder = os.path.join(raw_root, args.folder)
        if not os.path.isdir(target_folder):
            print(f"Error: Subfolder '{args.folder}' not found in {raw_root}")
            sys.exit(1)
        source_dirs = [target_folder]
    else:
        source_dirs = [os.path.join(raw_root, d) for d in os.listdir(raw_root) if os.path.isdir(os.path.join(raw_root, d))]

    print(f"Found {len(source_dirs)} folders to process.")
    
    for source_folder in source_dirs:
        folder_name = os.path.basename(source_folder)
        output_folder = os.path.join(post_processing_root, folder_name)
        os.makedirs(output_folder, exist_ok=True)
        
        print(f"\n--- Processing: {folder_name} ---")
        
        parquet_files = [f for f in sorted(os.listdir(source_folder)) if f.endswith(".parquet")]
        
        if not parquet_files:
            print("  No parquet files found.")
            continue
            
        for fname in parquet_files:
            print(f"  > {fname} ...")
            process_file(os.path.join(source_folder, fname), output_folder, show_plot=False)

if __name__ == "__main__":
    main()