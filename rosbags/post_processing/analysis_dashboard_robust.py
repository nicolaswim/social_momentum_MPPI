#!/usr/bin/env python3

## Component 2: Social Navigation Analysis Dashboard (TITANIUM 2.0)
##
## UPDATES:
## - Fixed Safety Monitor (Legend + messy lines).
## - Embedded "Failure Snapshots" (Dynamic Subplots in Bottom-Right).
## - Layout: 3-Column (Graphs Left/Mid, Report Card Right).
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.patches import Ellipse
import sys
import os
import argparse
import csv

# --- CONFIG ---
MAX_POS_RANGE = 5000.0 
HALL_X_MIN, HALL_X_MAX = -1.0, 12.0
HALL_Y_MIN, HALL_Y_MAX = -3.0, 3.0 
ELLIPSE_A = 1.2  
ELLIPSE_B = 0.6  
COLOR_RUDE = '#8B0000'    
COLOR_POLITE = '#90EE90'  
COLOR_ROBOT = 'blue'

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
    
    # Trim startup
    if not df.empty:
        start_time = df.index.min()
        cutoff = start_time + pd.Timedelta(seconds=20.1)
        df = df[df.index >= cutoff]
        if df.empty: return None

    if "pos_x" in df.columns:
        df = df[df["pos_x"].abs() < MAX_POS_RANGE]

    return df

def calculate_social_metrics(df):
    stats = {}
    
    # 1. KINEMATICS
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds().fillna(0)
    df_kin = df[df["delta_t"] > 0.0001].copy()
    
    df_kin["accel"] = df_kin["lin_vel"].diff() / df_kin["delta_t"]
    df_kin["jerk"] = df_kin["accel"].diff() / df_kin["delta_t"]
    stats["smoothness_score"] = df_kin["jerk"].abs().rolling(10).mean().mean()

    # 2. EFFICIENCY
    path_dist = np.sqrt(df["pos_x"].diff()**2 + df["pos_y"].diff()**2).sum()
    start_pos = np.array([df["pos_x"].iloc[0], df["pos_y"].iloc[0]])
    end_pos = np.array([df["pos_x"].iloc[-1], df["pos_y"].iloc[-1]])
    optimal_dist = np.linalg.norm(end_pos - start_pos)
    
    stats["path_length"] = path_dist
    stats["pir"] = path_dist / optimal_dist if optimal_dist > 0 else 0

    # 3. SOCIAL METRICS
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    human_prefixes = [c.replace("_x", "") for c in human_x_cols]
    
    events_metadata = [] 
    
    if human_prefixes:
        min_dists = []
        min_ttcs = []
        
        rx = df["gt_robot_x"].values if "gt_robot_x" in df.columns else df["pos_x"].values
        ry = df["gt_robot_y"].values if "gt_robot_y" in df.columns else df["pos_y"].values
        rvx = np.gradient(rx)
        rvy = np.gradient(ry)

        for h in human_prefixes:
            hx = df[f"{h}_x"].values
            hy = df[f"{h}_y"].values
            hvx = np.gradient(hx)
            hvy = np.gradient(hy)
            h_yaw = np.arctan2(hvy, hvx)
            
            dx = rx - hx; dy = ry - hy
            dist = np.sqrt(dx**2 + dy**2)
            df[f"dist_to_{h}"] = dist
            min_dists.append(dist)
            
            # Ellipse Check
            x_local = dx * np.cos(-h_yaw) - dy * np.sin(-h_yaw)
            y_local = dx * np.sin(-h_yaw) + dy * np.cos(-h_yaw)
            in_ellipse = ((x_local / ELLIPSE_A)**2 + (y_local / ELLIPSE_B)**2) <= 1
            
            # TTC Check
            vx_rel = rvx - hvx; vy_rel = rvy - hvy
            dot_prod = (dx * vx_rel) + (dy * vy_rel)
            speed_rel_towards = dot_prod / (dist + 0.001)
            
            ttc = np.full_like(dist, 10.0) 
            closing_mask = speed_rel_towards > 0.05 
            ttc[closing_mask] = dist[closing_mask] / speed_rel_towards[closing_mask]
            
            # Static Danger Fix
            robot_speed = np.sqrt(rvx**2 + rvy**2)
            static_danger_mask = (dist < 0.5) & (robot_speed < 0.05)
            ttc[static_danger_mask] = 0.1 
            
            min_ttcs.append(np.min(ttc))

            # Event Tracking
            states = in_ellipse.astype(int)
            transitions = np.diff(states, prepend=0)
            starts = np.where(transitions == 1)[0]
            ends = np.where(transitions == -1)[0]
            
            if states[0] == 1: starts = np.insert(starts, 0, 0)
            if len(ends) < len(starts): ends = np.append(ends, len(states)-1)
            
            for s, e in zip(starts, ends):
                event_dists = dist[s:e+1]
                local_min_idx = np.argmin(event_dists)
                global_idx = s + local_min_idx
                
                events_metadata.append({
                    "human": h,
                    "frame_idx": global_idx,
                    "min_dist": dist[global_idx],
                    "rx": rx[global_idx], "ry": ry[global_idx],
                    "hx": hx[global_idx], "hy": hy[global_idx],
                    "rvx": rvx[global_idx], "rvy": rvy[global_idx],
                    "hvx": hvx[global_idx], "hvy": hvy[global_idx],
                    "h_yaw": h_yaw[global_idx]
                })

        df["nearest_human_dist"] = np.min(min_dists, axis=0)
        
        stats["min_human_dist"] = np.min(df["nearest_human_dist"])
        stats["min_ttc"] = np.min(min_ttcs) if min_ttcs else 10.0
        
        # Sort events by severity (closest distance first)
        events_metadata.sort(key=lambda x: x["min_dist"])
        stats["events"] = events_metadata
        stats["intrusion_count"] = len(events_metadata)
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
        "pir": stats.get("pir", 0),
        "smoothness_score": stats.get("smoothness_score", 0),
        "min_human_dist": stats.get("min_human_dist", -1),
        "min_ttc": stats.get("min_ttc", 999),
        "intrusion_count": stats.get("intrusion_count", 0),
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
    
    # --- LAYOUT: 2 Rows, 3 Columns ---
    # Col 0, 1: Graphs (Map, Safety, Polite, Failures)
    # Col 2: Report Card (Sidebar)
    fig = plt.figure(figsize=(24, 12))
    gs = gridspec.GridSpec(2, 3, width_ratios=[1, 1, 0.4]) 
    fig.suptitle(f"Social Nav Analysis: {os.path.basename(filename)}", fontsize=16)

    # ==========================================================
    # 1. TOP LEFT: Global Map
    # ==========================================================
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title("Hallway Activity Map (Global)")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
    ax1.axhline(y=2.5, color='k', linewidth=3)
    ax1.axhline(y=-2.5, color='k', linewidth=3)
    ax1.set_xlim(HALL_X_MIN, HALL_X_MAX)
    ax1.set_ylim(HALL_Y_MIN, HALL_Y_MAX)
    
    # Humans
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    for h_col in human_x_cols:
        h_prefix = h_col.replace("_x", "")
        valid_mask = df[h_col].abs() < 50
        ax1.plot(df.loc[valid_mask, f"{h_prefix}_x"].values, 
                 df.loc[valid_mask, f"{h_prefix}_y"].values, 
                 color="grey", alpha=0.3)

    # Robot
    rx = df["gt_robot_x"].values if "gt_robot_x" in df.columns else df["pos_x"].values
    ry = df["gt_robot_y"].values if "gt_robot_y" in df.columns else df["pos_y"].values
    ax1.plot(rx, ry, color=COLOR_ROBOT, label="Robot Path", linewidth=2)
    ax1.plot(rx[0], ry[0], "go"); ax1.plot(rx[-1], ry[-1], "rx")
    
    # Markers (Top 4 events only to keep clean)
    if stats.get("events"):
        for i, event in enumerate(stats["events"][:4]):
            ax1.plot(event["rx"], event["ry"], 'rX', markersize=10)
            ax1.text(event["rx"], event["ry"]+0.3, f"X{i+1}", color='red', fontweight='bold')
    
    ax1.legend(loc="upper right")
    ax1.grid(True, linestyle=':')

    # ==========================================================
    # 2. TOP MIDDLE: Safety Monitor (Fixed Legend & Lines)
    # ==========================================================
    ax2 = fig.add_subplot(gs[0, 1])
    if stats["has_social_data"]:
        ax2.set_title("Safety Monitor: Distance & TTC")
        ax2.set_xlabel("Time (HH:MM:SS)")
        ax2.set_ylabel("Distance (m)", color='orange')
        
        # Distance (Left Axis)
        l1 = ax2.plot(df.index.values, df["nearest_human_dist"].values, color="orange", label="Dist")
        ax2.axhline(y=0.45, color='red', linestyle='--', alpha=0.5, label="Intimate Limit")
        ax2.tick_params(axis='y', labelcolor='orange')
        
        # TTC (Right Axis)
        ax2_r = ax2.twinx()
        ax2_r.set_ylabel("TTC (s)", color='purple')
        
        # Calc Plot-Friendly TTC (Clipped)
        ttc_plot = df["nearest_human_dist"] / (df["lin_vel"] + 0.01)
        ttc_plot = ttc_plot.clip(upper=10.0) 
        
        l2 = ax2_r.plot(df.index.values, ttc_plot.values, color='purple', alpha=0.3, label="Est. TTC")
        ax2_r.tick_params(axis='y', labelcolor='purple')
        ax2_r.set_ylim(0, 10)
        
        # COMBINED LEGEND
        lines = l1 + l2
        labels = [l.get_label() for l in lines]
        ax2.legend(lines, labels, loc="upper right")
    else:
        ax2.text(0.5, 0.5, "NO DATA", ha='center')
    ax2.grid(True)

    # ==========================================================
    # 3. BOTTOM LEFT: Politeness
    # ==========================================================
    ax3 = fig.add_subplot(gs[1, 0])
    slope_status = "N/A"; slope_val = 0.0
    
    if stats["has_social_data"]:
        ax3.set_title("Politeness (Cleaned 5-95%)")
        ax3.set_xlabel("Dist (m)"); ax3.set_ylabel("Vel (m/s)")
        ax3.set_ylim(-0.05, 0.6)
        
        # Zones
        x_zone = np.linspace(0, 10, 100)
        ax3.fill_between(x_zone, 0.1*x_zone, 2.0, color=COLOR_POLITE, alpha=0.2, label="Polite")
        ax3.fill_between(x_zone, -0.1, 0.1*x_zone, color=COLOR_RUDE, alpha=0.2, label="Rude")

        # Scatter
        x = df["nearest_human_dist"].values; y = df["lin_vel"].values
        mask = ~np.isnan(x) & ~np.isnan(y)
        x_c = x[mask]; y_c = y[mask]
        
        if len(x_c) > 10:
            xl, xh = np.percentile(x_c, [5, 95])
            yl, yh = np.percentile(y_c, [5, 95])
            m_c = (x_c > xl) & (x_c < xh) & (y_c > yl) & (y_c < yh)
            xf, yf = x_c[m_c], y_c[m_c]
            
            ax3.scatter(xf, yf, color='gray', alpha=0.3, s=5)
            
            if len(xf) > 5:
                z = np.polyfit(xf, yf, 1)
                slope_val = z[0]
                p = np.poly1d(z)
                col = "green" if slope_val > 0.1 else "red" if slope_val < 0.05 else "orange"
                slope_status = "POLITE" if slope_val > 0.1 else "RUDE" if slope_val < 0.05 else "NEUTRAL"
                ax3.plot(np.linspace(xf.min(), xf.max(), 100), p(np.linspace(xf.min(), xf.max(), 100)), 
                         color=col, linewidth=3, label=f"Trend (m={slope_val:.2f})")
                ax3.legend(loc="upper left")
    else:
        ax3.text(0.5, 0.5, "NO DATA", ha='center')
    
    stats["politeness_slope"] = slope_val
    stats["politeness_status"] = slope_status
    ax3.grid(True)

    # ==========================================================
    # 4. BOTTOM MIDDLE: Dynamic Failure Snapshots (Embedded)
    # ==========================================================
    # Create a nested GridSpec inside the gs[1,1] cell
    
    # How many events to show? Max 4.
    events = stats.get("events", [])
    num_show = min(len(events), 4)
    
    if num_show > 0:
        # Define layout (1x1, 1x2, or 2x2)
        rows = 1 if num_show <= 2 else 2
        cols = num_show if num_show <= 2 else 2
        
        gs_inner = gridspec.GridSpecFromSubplotSpec(rows, cols, subplot_spec=gs[1, 1], wspace=0.3, hspace=0.4)
        
        for i in range(num_show):
            ax_sub = fig.add_subplot(gs_inner[i])
            ev = events[i]
            
            ax_sub.set_title(f"X{i+1}: Dist {ev['min_dist']:.2f}m", fontsize=9)
            ax_sub.set_xlim(ev["rx"] - 2.0, ev["rx"] + 2.0)
            ax_sub.set_ylim(ev["ry"] - 2.0, ev["ry"] + 2.0)
            ax_sub.axis('off') # Cleaner look
            
            # Draw Ellipse
            deg = np.degrees(ev["h_yaw"])
            ell = Ellipse((ev["hx"], ev["hy"]), width=ELLIPSE_A*2, height=ELLIPSE_B*2, 
                          angle=deg, color='r', alpha=0.2)
            ax_sub.add_patch(ell)
            
            # Draw Agents
            ax_sub.plot(ev["hx"], ev["hy"], 'ro', markersize=6)
            ax_sub.arrow(ev["hx"], ev["hy"], ev["hvx"], ev["hvy"], head_width=0.1, color='r', length_includes_head=True)
            
            ax_sub.plot(ev["rx"], ev["ry"], 'bo', markersize=6)
            ax_sub.arrow(ev["rx"], ev["ry"], ev["rvx"], ev["rvy"], head_width=0.1, color='b', length_includes_head=True)
            
            # Circle Robot
            ax_sub.add_patch(plt.Circle((ev["rx"], ev["ry"]), 0.3, color='b', fill=False))
            
    else:
        ax_sub = fig.add_subplot(gs[1, 1])
        ax_sub.text(0.5, 0.5, "No Social Violations\nSafe Run", ha='center', fontsize=12)
        ax_sub.axis('off')


    # ==========================================================
    # 5. RIGHT SIDEBAR: Report Card
    # ==========================================================
    ax5 = fig.add_subplot(gs[:, 2])
    ax5.axis("off")
    
    lines = [
        "--- RUN SUMMARY ---",
        f"Time:       {stats['duration']:.2f} s",
        f"Path Len:   {stats['path_length']:.2f} m",
        f"Smoothness: {stats['smoothness_score']:.2f} m/s^3",
        "            (< 5 is smooth)",
        f"PIR:        {stats['pir']:.2f} (1.0=Opt)",
        "",
        "--- SOCIAL ANALYSIS ---",
    ]
    
    if stats["has_social_data"]:
        lines.append(f"Min Dist:   {stats['min_human_dist']:.2f} m")
        lines.append(f"Min TTC:    {stats['min_ttc']:.2f} s")
        lines.append(f"Violations: {stats['intrusion_count']}")
        lines.append("")
        lines.append(f"Politeness: {slope_status}")
        lines.append(f"Slope:      {slope_val:.3f}")
        
        status = "FAIL" if (stats['min_human_dist'] < 0.45 or stats['min_ttc'] < 0.5) else "PASS"
        lines.append("")
        lines.append(f"RESULT:     {status}")
        
        if num_show > 0:
            lines.append("")
            lines.append("--- VIOLATIONS ---")
            for i in range(num_show):
                ev = events[i]
                lines.append(f"X{i+1}: {ev['human']} ({ev['min_dist']:.2f}m)")

    else:
        lines.append("No Human Data")

    ax5.text(0.05, 0.95, "\n\n".join(lines), transform=ax5.transAxes, 
             fontsize=14, family="monospace", va="top")

    # Save
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