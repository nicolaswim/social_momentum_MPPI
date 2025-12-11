#!/usr/bin/env python3

## Component 2: Offline Analysis and Visualization (BULLETPROOF)
##
## Repairs:
## - Filters "Space Junk" coordinates (values > 10km).
## - Clamps acceleration/jerk.
## - Safe plotting blocks (skips individual plots if they fail, doesn't crash script).
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import argparse

# --- PHYSICS LIMITS ---
MAX_ACCEL = 50.0      # m/s^2
MAX_JERK = 500.0      # m/s^3
MAX_POS_RANGE = 5000.0 # Meters (Discard points > 5km away)
MAX_VEL = 50.0        # m/s

def load_data(parquet_file):
    if not os.path.exists(parquet_file):
        print(f"File not found: {parquet_file}")
        return None
        
    print(f"Loading data from {parquet_file}...")
    try:
        df = pd.read_parquet(parquet_file, engine="pyarrow")
    except Exception as e:
        print(f"[ERROR] Corrupt Parquet file: {e}")
        return None
    
    ACTUAL_TIME_COLUMN = "timestamp"
    if ACTUAL_TIME_COLUMN not in df.columns:
        cols = [c for c in df.columns if "time" in c.lower()]
        if cols:
            ACTUAL_TIME_COLUMN = cols[0]
        else:
            print(f"[ERROR] Timestamp column missing.")
            return None
        
    df = df.drop_duplicates()

    if pd.api.types.is_numeric_dtype(df[ACTUAL_TIME_COLUMN]):
            print(f"[INFO] Converting numeric timestamp (nanoseconds) to datetime objects...")
            df[ACTUAL_TIME_COLUMN] = pd.to_datetime(df[ACTUAL_TIME_COLUMN], unit='ns')

    df.set_index(ACTUAL_TIME_COLUMN, inplace=True)
    df.sort_index(inplace=True)
    df = df[~df.index.duplicated(keep='first')]
    
    # --- STAGE 1: SANITIZE INPUTS ---
    # Drop rows with garbage floating point values (NaN, Inf, or Massive)
    
    # 1. Sanitize Position (Robot didn't teleport to space)
    if "pos_x" in df.columns and "pos_y" in df.columns:
        df = df[df["pos_x"].abs() < MAX_POS_RANGE]
        df = df[df["pos_y"].abs() < MAX_POS_RANGE]

    # 2. Sanitize Velocity
    if "lin_vel" in df.columns:
        df = df[df["lin_vel"].abs() < MAX_VEL]

    print(f"Loaded {len(df)} clean data points.")
    return df

def calculate_metrics(df):
    print("Calculating metrics...")
    
    if len(df) < 2: return df, {}

    # Time diff
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds()
    
    # Filter invalid time steps
    df = df[df["delta_t"] > 0.0001].copy()

    # Calculate Derivatives
    df["acceleration_m_s2"] = df["lin_vel"].diff() / df["delta_t"]
    df["jerk_m_s3"] = df["acceleration_m_s2"].diff() / df["delta_t"]

    # --- STAGE 2: SANITIZE OUTPUTS ---
    
    # 1. Replace Inf with NaN
    df.replace([np.inf, -np.inf], np.nan, inplace=True)
    
    # 2. Drop NaNs
    df.dropna(subset=["acceleration_m_s2", "jerk_m_s3"], inplace=True)
    
    # 3. Clamp Physics
    df = df[df["acceleration_m_s2"].abs() < MAX_ACCEL]
    df = df[df["jerk_m_s3"].abs() < MAX_JERK]

    if df.empty:
        return df, {"total_duration": 0, "avg_joint_speeds": {}}

    # KPIs
    total_duration = (df.index.max() - df.index.min()).total_seconds()
    
    joint_vel_cols = [col for col in df.columns if col.endswith("_vel") and col != "lin_vel"]
    avg_joint_speeds = df[joint_vel_cols].abs().mean() if joint_vel_cols else {}
    
    kpi_summary = {
        "total_duration": total_duration,
        "avg_joint_speeds": avg_joint_speeds,
    }
    
    return df, kpi_summary

def create_dashboard(df, kpi_summary, parquet_file, output_dir, show_plot=True):
    if len(df) < 10:
        print("Skipping dashboard (Insufficient Data after filtering)")
        return

    print("Generating dashboard...")
    
    try:
        fig, axs = plt.subplots(2, 2, figsize=(18, 12))
        fig.suptitle(f"Robot Performance: {os.path.basename(parquet_file)}", fontsize=16)

        # --- Plot 1: Path ---
        try:
            ax1 = axs[0, 0]
            ax1.plot(df["pos_x"].values, df["pos_y"].values, color="blue", label="Path", linewidth=1)
            ax1.plot(df["pos_x"].iloc[0], df["pos_y"].iloc[0], "go", markersize=8, label="Start")
            ax1.plot(df["pos_x"].iloc[-1], df["pos_y"].iloc[-1], "rs", markersize=8, label="End")
            ax1.set_title("Robot Path (Odom)")
            ax1.set_xlabel("X (m)")
            ax1.set_ylabel("Y (m)")
            ax1.legend()
            ax1.grid(True)
            ax1.axis("equal")
        except Exception as e:
            print(f"[WARN] Plot 1 failed: {e}")

        # --- Plot 2: Kinematics ---
        try:
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
        except Exception as e:
            print(f"[WARN] Plot 2 failed: {e}")

        # --- Plot 3: Joints ---
        try:
            ax3 = axs[1, 0]
            joint_cols = [c for c in df.columns if c.endswith("_vel") and c != "lin_vel"]
            if joint_cols:
                top_joints = df[joint_cols].abs().max().nlargest(5).index.tolist()
                for col in top_joints:
                    ax3.plot(x_axis, df[col].values, label=col.replace("_joint_vel",""), alpha=0.8)
                ax3.legend(loc="upper right", fontsize='small')
            ax3.set_title("Top 5 Active Joints")
            ax3.set_xlabel("Sample")
            ax3.set_ylabel("Rad/s")
            ax3.grid(True)
        except Exception as e:
            print(f"[WARN] Plot 3 failed: {e}")

        # --- Plot 4: Stats ---
        ax4 = axs[1, 1]
        ax4.axis("off")
        
        dur = kpi_summary.get("total_duration", 0)
        text_lines = [
            "--- STATISTICS ---",
            f"Duration: {dur:.2f} sec",
            f"Samples:  {len(df)}",
            "",
            "--- AVG JOINT VEL (rad/s) ---"
        ]
        
        avgs = kpi_summary.get("avg_joint_speeds", {})
        if isinstance(avgs, pd.Series):
            for joint, speed in avgs.items():
                text_lines.append(f"{joint.replace('_joint_vel',''):<20}: {speed:.3f}")

        ax4.text(0.05, 0.95, "\n".join(text_lines), transform=ax4.transAxes, 
                 fontsize=10, verticalalignment="top", fontfamily="monospace")

        fig.tight_layout(rect=[0, 0.03, 1, 0.96])
        
        file_name = os.path.splitext(os.path.basename(parquet_file))[0]
        output_image = os.path.join(output_dir, file_name + ".png")
        
        plt.savefig(output_image)
        print(f"Dashboard saved: {output_image}")
        plt.close(fig)
        
    except Exception as e:
        print(f"[ERROR] Dashboard generation failed: {e}")

def process_file(parquet_file, output_dir, show_plot):
    try:
        df = load_data(parquet_file)
        if df is not None:
            df_metrics, kpi_summary = calculate_metrics(df)
            create_dashboard(df_metrics, kpi_summary, parquet_file, output_dir, show_plot)
    except Exception as e:
        print(f"[ERROR] Failed to process {parquet_file}: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root_dir", help="Path to raw data (e.g. rosbags/raw)")
    parser.add_argument("--folder", help="Specific subfolder", default=None)
    args = parser.parse_args()
    
    raw_root = os.path.abspath(args.root_dir)
    if not os.path.isdir(raw_root):
        print(f"Error: {raw_root} is not a directory.")
        sys.exit(1)

    rosbags_dir = os.path.dirname(raw_root)
    post_processing_root = os.path.join(rosbags_dir, "post_processing")

    if args.folder:
        target = os.path.join(raw_root, args.folder)
        if not os.path.isdir(target):
            print(f"Folder not found: {target}")
            sys.exit(1)
        source_dirs = [target]
    else:
        source_dirs = [os.path.join(raw_root, d) for d in os.listdir(raw_root) if os.path.isdir(os.path.join(raw_root, d))]

    print(f"Processing {len(source_dirs)} folders...")
    
    for source in source_dirs:
        folder_name = os.path.basename(source)
        out_dir = os.path.join(post_processing_root, folder_name)
        os.makedirs(out_dir, exist_ok=True)
        
        print(f"\n--- {folder_name} ---")
        files = [f for f in sorted(os.listdir(source)) if f.endswith(".parquet")]
        
        if not files:
            print("  No parquet files.")
            continue
            
        for f in files:
            print(f"  > {f} ...")
            process_file(os.path.join(source, f), out_dir, False)

if __name__ == "__main__":
    main()