#!/usr/bin/env python3

## Component 2: Social Navigation Analysis Dashboard (DIAMOND 2.0)
##
## UPDATES:
## - 20.1s Start Trim: Ignores the first 20.1 seconds of simulation setup.
## - Visuals: Removed Vision Lines (Arrows).
## - Colors: Dark Red (Rude) / Light Green (Polite).
## - Labels: Fixed X-Axis label on Top Right graph.
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import sys
import os
import argparse
import csv

# --- PHYSICS LIMITS ---
MAX_POS_RANGE = 5000.0 

# --- HALLWAY DIMENSIONS ---
HALL_X_MIN, HALL_X_MAX = -1.0, 12.0
HALL_Y_MIN, HALL_Y_MAX = -3.0, 3.0 

# --- COLORS ---
COLOR_RUDE_ZONE = '#8B0000'    # Dark Red
COLOR_POLITE_ZONE = '#90EE90'  # Light Green
COLOR_ROBOT_PATH = 'blue'

def load_data(parquet_file):
    if not os.path.exists(parquet_file): return None
    print(f"Loading {parquet_file}...")
    try:
        df = pd.read_parquet(parquet_file, engine="pyarrow")
    except Exception as e:
        print(f"[ERROR] Corrupt file: {e}")
        return None
    
    time_col = "timestamp"
    if time_col not in df.columns:
        cols = [c for c in df.columns if "time" in c.lower()]
        time_col = cols[0] if cols else None
    if not time_col: return None

    df = df.drop_duplicates()
    if pd.api.types.is_numeric_dtype(df[time_col]):
        df[time_col] = pd.to_datetime(df[time_col], unit='ns')

    df.set_index(time_col, inplace=True)
    df.sort_index(inplace=True)
    
    # --- STEP 1: TRIM STARTUP TIME (20.1s) ---
    if not df.empty:
        start_time = df.index.min()
        cutoff = start_time + pd.Timedelta(seconds=20.1)
        original_len = len(df)
        df = df[df.index >= cutoff]
        new_len = len(df)
        # print(f"  > Trimmed startup: Dropped {original_len - new_len} frames (First 20.1s)")
        
        if df.empty:
            print("  [WARN] Datafile empty after trimming 20.1s. Skipping.")
            return None

    if "pos_x" in df.columns:
        df = df[df["pos_x"].abs() < MAX_POS_RANGE]

    return df

def calculate_social_metrics(df):
    stats = {}
    
    # 1. Kinematics
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds().fillna(0)
    df_kin = df[df["delta_t"] > 0.0001].copy()
    
    df_kin["accel"] = df_kin["lin_vel"].diff() / df_kin["delta_t"]
    df_kin["jerk"] = df_kin["accel"].diff() / df_kin["delta_t"]
    stats["smoothness_score"] = df_kin["jerk"].abs().rolling(10).mean().mean()

    # 2. Path Length
    dist = np.sqrt(df["pos_x"].diff()**2 + df["pos_y"].diff()**2)
    stats["path_length"] = dist.sum()
    
    # 3. Social Metrics
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    human_prefixes = [c.replace("_x", "") for c in human_x_cols]
    
    if human_prefixes:
        dist_cols = []
        total_events = 0
        total_danger_duration = 0.0
        
        times = df.index.astype(np.int64) / 1e9
        
        for h in human_prefixes:
            rx = df["gt_robot_x"] if "gt_robot_x" in df.columns else df["pos_x"]
            ry = df["gt_robot_y"] if "gt_robot_y" in df.columns else df["pos_y"]
            hx = df[f"{h}_x"]
            hy = df[f"{h}_y"]
            
            d_series = np.sqrt((rx - hx)**2 + (ry - hy)**2)
            df[f"dist_to_{h}"] = d_series
            dist_cols.append(f"dist_to_{h}")

            # State Machine for Intrusions
            states = (d_series < 0.45).astype(int).values
            transitions = np.diff(states, prepend=0)
            
            enter_indices = np.where(transitions == 1)[0]
            exit_indices = np.where(transitions == -1)[0]
            
            if states[0] == 1:
                enter_indices = np.insert(enter_indices, 0, 0)
            if len(exit_indices) < len(enter_indices):
                exit_indices = np.append(exit_indices, len(states) - 1)
                
            total_events += len(enter_indices)
            for start, end in zip(enter_indices, exit_indices):
                total_danger_duration += times[end] - times[start]

        df["nearest_human_dist"] = df[dist_cols].min(axis=1)
        
        stats["min_human_dist"] = df["nearest_human_dist"].min()
        stats["avg_human_dist"] = df["nearest_human_dist"].mean()
        stats["intrusion_count"] = total_events
        stats["time_in_danger"] = total_danger_duration
        stats["has_social_data"] = True
    else:
        stats["min_human_dist"] = -1
        stats["has_social_data"] = False

    stats["duration"] = (df.index.max() - df.index.min()).total_seconds()
    return df, stats

def save_csv_per_run(stats, filename, output_dir):
    csv_filename = os.path.splitext(os.path.basename(filename))[0] + ".csv"
    csv_path = os.path.join(output_dir, csv_filename)
    
    row = {
        "filename": filename,
        "duration": stats.get("duration", 0),
        "path_length": stats.get("path_length", 0),
        "smoothness_score": stats.get("smoothness_score", 0),
        "min_human_dist": stats.get("min_human_dist", -1),
        "avg_human_dist": stats.get("avg_human_dist", -1),
        "intrusion_count": stats.get("intrusion_count", 0),
        "time_in_danger": stats.get("time_in_danger", 0),
        "politeness_slope": stats.get("politeness_slope", 0),
        "politeness_status": stats.get("politeness_status", "N/A")
    }
    
    try:
        with open(csv_path, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            writer.writeheader()
            writer.writerow(row)
        print(f"[CSV] Saved metrics to {csv_path}")
    except Exception as e:
        print(f"[CSV ERROR] {e}")

def create_dashboard(df, stats, filename, output_dir):
    print("Generating dashboard...")
    
    fig = plt.figure(figsize=(22, 12))
    gs = gridspec.GridSpec(2, 3, width_ratios=[1, 1, 0.6]) 
    fig.suptitle(f"Social Nav Analysis: {os.path.basename(filename)}", fontsize=16)

    # ==========================================================
    # 1. TOP LEFT: Hallway Map
    # ==========================================================
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title("Hallway Activity Map")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.axhline(y=2.5, color='k', linewidth=3)
    ax1.axhline(y=-2.5, color='k', linewidth=3)
    ax1.set_xlim(HALL_X_MIN, HALL_X_MAX)
    ax1.set_ylim(HALL_Y_MIN, HALL_Y_MAX)
    
    # Plot Humans
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    added_legend = False
    for h_col in human_x_cols:
        h_prefix = h_col.replace("_x", "")
        valid_mask = df[h_col].abs() < 50
        label = "Humans" if not added_legend else ""
        ax1.plot(df.loc[valid_mask, f"{h_prefix}_x"].values, 
                 df.loc[valid_mask, f"{h_prefix}_y"].values, 
                 color="grey", alpha=0.3, label=label)
        if not added_legend: added_legend = True

    # Plot Robot (No Quiver/Arrows anymore)
    rx = df["gt_robot_x"].values if "gt_robot_x" in df.columns else df["pos_x"].values
    ry = df["gt_robot_y"].values if "gt_robot_y" in df.columns else df["pos_y"].values
    ax1.plot(rx, ry, color=COLOR_ROBOT_PATH, label="Robot", linewidth=2)
    ax1.plot(rx[0], ry[0], "go"); ax1.plot(rx[-1], ry[-1], "rx")
    
    ax1.legend(loc="upper right")
    ax1.grid(True, linestyle=':')

    # ==========================================================
    # 2. TOP MIDDLE: Safety Monitor
    # ==========================================================
    ax2 = fig.add_subplot(gs[0, 1])
    if stats["has_social_data"]:
        ax2.set_title("Safety Monitor")
        ax2.set_ylabel("Dist (m)")
        # FIX: ADDED LABEL
        ax2.set_xlabel("Time (HH:MM:SS)")
        
        ax2.plot(df.index.values, df["nearest_human_dist"].values, color="orange", label="Nearest Human")
        ax2.axhline(y=0.45, color='r', linestyle='--', label="Intimate (0.45m)")
        
        failures = df[df["nearest_human_dist"] < 0.45]
        if not failures.empty:
            ax2.plot(failures.index.values, failures["nearest_human_dist"].values, "rx", label="Intrusion")
        ax2.legend(loc="upper right")
    else:
        ax2.text(0.5, 0.5, "NO DATA", ha='center')
    ax2.grid(True)

    # ==========================================================
    # 3. BOTTOM LEFT: Raw Politeness
    # ==========================================================
    ax3 = fig.add_subplot(gs[1, 0])
    if stats["has_social_data"]:
        ax3.set_title("Raw Data: Velocity vs. Proximity")
        ax3.set_xlabel("Dist to Human (m)")
        ax3.set_ylabel("Vel (m/s)")
        ax3.scatter(df["nearest_human_dist"].values, df["lin_vel"].values, 
                    c=np.arange(len(df)), cmap="Blues", alpha=0.3, s=5)
    else:
        ax3.text(0.5, 0.5, "NO DATA", ha='center')
    ax3.grid(True)

    # ==========================================================
    # 4. BOTTOM MIDDLE: Cleaned Trend (With Colors)
    # ==========================================================
    ax4 = fig.add_subplot(gs[1, 1])
    slope_status = "N/A"
    slope_val = 0.0
    
    if stats["has_social_data"]:
        ax4.set_title("Cleaned Trend (5-95%ile)")
        ax4.set_xlabel("Dist to Human (m)")
        ax4.set_ylim(-0.05, 0.6) 
        
        # --- DRAW ZONES (UPDATED COLORS) ---
        x_zone = np.linspace(0, 10, 100)
        y_thresh = 0.1 * x_zone 
        
        # Polite = Light Green
        ax4.fill_between(x_zone, y_thresh, 2.0, color=COLOR_POLITE_ZONE, alpha=0.2, label="Polite Zone")
        # Rude = Dark Red
        ax4.fill_between(x_zone, -0.1, y_thresh, color=COLOR_RUDE_ZONE, alpha=0.2, label="Rude Zone")

        # Data processing
        x = df["nearest_human_dist"].values
        y = df["lin_vel"].values
        mask = ~np.isnan(x) & ~np.isnan(y)
        x_clean = x[mask]; y_clean = y[mask]
        
        if len(x_clean) > 10:
            x_low, x_high = np.percentile(x_clean, [5, 95])
            y_low, y_high = np.percentile(y_clean, [5, 95])
            
            clean_mask = (x_clean > x_low) & (x_clean < x_high) & \
                         (y_clean > y_low) & (y_clean < y_high)
            xf = x_clean[clean_mask]; yf = y_clean[clean_mask]
            
            ax4.scatter(xf, yf, color='gray', alpha=0.1, s=5)
            
            if len(xf) > 5:
                z = np.polyfit(xf, yf, 1)
                slope_val = z[0]
                p = np.poly1d(z)
                
                if slope_val > 0.1:
                    line_color = "green"; slope_status = "POLITE"
                elif slope_val < 0.05:
                    line_color = "red"; slope_status = "RUDE"
                else:
                    line_color = "orange"; slope_status = "NEUTRAL"

                x_line = np.linspace(xf.min(), xf.max(), 100)
                ax4.plot(x_line, p(x_line), color=line_color, linewidth=3, 
                         label=f"Trend (m={slope_val:.2f})")
                ax4.legend(loc="upper left")
    else:
        ax4.text(0.5, 0.5, "NO DATA", ha='center')
    
    stats["politeness_slope"] = slope_val
    stats["politeness_status"] = slope_status
    ax4.grid(True)

    # ==========================================================
    # 5. RIGHT COLUMN: Report Card
    # ==========================================================
    ax5 = fig.add_subplot(gs[:, 2])
    ax5.axis("off")
    
    lines = [
        "--- RUN SUMMARY ---",
        f"Time:     {stats['duration']:.2f} s",
        f"Path:     {stats['path_length']:.2f} m",
        f"Smoothness: {stats['smoothness_score']:.2f} m/s^3",
        "",
        "--- SOCIAL ANALYSIS ---",
    ]
    
    if stats["has_social_data"]:
        lines.append(f"Min Dist:   {stats['min_human_dist']:.2f} m")
        lines.append(f"Avg Dist:   {stats['avg_human_dist']:.2f} m")
        lines.append("")
        lines.append(f"Intrusions: {stats['intrusion_count']} Events")
        lines.append(f"Danger Time: {stats['time_in_danger']:.2f} s")
        lines.append("")
        lines.append(f"Politeness: {slope_status}")
        lines.append(f"Slope:      {slope_val:.3f}")
    else:
        lines.append("No Human Data")

    ax5.text(0.1, 0.8, "\n\n".join(lines), transform=ax5.transAxes, 
             fontsize=14, family="monospace", va="top")

    plt.tight_layout()
    img_path = os.path.join(output_dir, os.path.splitext(os.path.basename(filename))[0] + ".png")
    plt.savefig(img_path)
    print(f"Saved Dashboard: {img_path}")
    plt.close(fig)
    
    save_csv_per_run(stats, filename, output_dir)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root_dir", help="Path to raw data")
    parser.add_argument("--folder", help="Specific subfolder", default=None)
    args = parser.parse_args()
    
    raw_root = os.path.abspath(args.root_dir)
    if os.path.basename(raw_root) == "raw":
        base_dir = os.path.dirname(raw_root)
        post_root = os.path.join(base_dir, "post_processing")
    else:
        post_root = os.path.join(raw_root, "post_processing")

    if args.folder:
        target = os.path.join(raw_root, args.folder)
        if not os.path.isdir(target):
            print(f"Error: Folder '{args.folder}' not found")
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
        
        for f in files:
            df = load_data(os.path.join(source, f))
            if df is not None and not df.empty:
                df, stats = calculate_social_metrics(df)
                create_dashboard(df, stats, f, out_dir)

if __name__ == "__main__":
    main()