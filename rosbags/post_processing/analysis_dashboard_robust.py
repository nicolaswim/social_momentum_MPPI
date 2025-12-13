#!/usr/bin/env python3

## Component 2: Social Navigation Analysis Dashboard (TITANIUM 8.7 - GLOBAL SUCCESS CUTOFF)
##
## UPDATES:
## - CRITICAL: Data is now truncated the moment the robot reaches within 35cm of (9,0).
## - CONSEQUENCE: Jerk, PIR, Politeness, Collisions, and Graphs ONLY consider data BEFORE success.
## - LOGIC: Exclusion zone (0.5m) is still there as a safety backup, though strictly less necessary now.
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.patches import Ellipse, Circle, Patch
from matplotlib.lines import Line2D
import sys
import os
import argparse
import csv

# --- CONFIG ---
MAX_POS_RANGE = 5000.0 
HALL_X_MIN, HALL_X_MAX = -1.0, 12.0
HALL_Y_MIN, HALL_Y_MAX = -3.0, 3.0 
ROBOT_RADIUS = 0.3
HUMAN_RADIUS = 0.3 

# --- GOAL / SUCCESS PARAMETERS ---
GOAL_X = 9.0
GOAL_Y = 0.0
GOAL_TOLERANCE = 0.35  # Robot "Succeeds" if within 35cm of goal

# --- SOCIAL PARAMETERS ---
ELLIPSE_A = 1.2  
ELLIPSE_B = 0.6  
DANGER_TTC_LIMIT = 2.0
MERGE_TIME_THRESHOLD = 8.0  
MERGE_DIST_THRESHOLD = 2.0  

# --- EXCLUSION ZONE PARAMETERS ---
EXCLUSION_X = 9.0
EXCLUSION_Y = 0.0
EXCLUSION_RADIUS = 0.5  

# --- COLORS ---
COLOR_RUDE_ZONE = '#D32F2F'     
COLOR_POLITE_ZONE = '#388E3C'   
COLOR_ROBOT = 'blue'
COLOR_DANGER_FILL = '#ffcccc'

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
    
    if not df.empty:
        start_time = df.index.min()
        cutoff = start_time + pd.Timedelta(seconds=3.1)
        df = df[df.index >= cutoff]
        if df.empty: return None

    if "pos_x" in df.columns:
        df = df[df["pos_x"].abs() < MAX_POS_RANGE]

    return df

def filter_events_spatial_temporal(raw_events, time_thresh=8.0, dist_thresh=2.0):
    if not raw_events: return []
    
    events_by_human = {}
    for ev in raw_events:
        h_id = ev['human']
        if h_id not in events_by_human:
            events_by_human[h_id] = []
        events_by_human[h_id].append(ev)
    
    final_unique_events = []

    for h_id, ev_list in events_by_human.items():
        ev_list.sort(key=lambda x: x['timestamp_val'])
        current_cluster = [ev_list[0]]
        
        for i in range(1, len(ev_list)):
            prev_frame = current_cluster[-1]
            curr_frame = ev_list[i]
            
            time_diff = curr_frame['timestamp_val'] - prev_frame['timestamp_val']
            dist_diff = np.sqrt((curr_frame['rx'] - prev_frame['rx'])**2 + 
                                (curr_frame['ry'] - prev_frame['ry'])**2)
            
            if time_diff < time_thresh or dist_diff < dist_thresh:
                current_cluster.append(curr_frame)
            else:
                worst_frame = min(current_cluster, key=lambda x: x['min_dist'])
                final_unique_events.append(worst_frame)
                current_cluster = [curr_frame]
        
        if current_cluster:
            worst_frame = min(current_cluster, key=lambda x: x['min_dist'])
            final_unique_events.append(worst_frame)
        
    return final_unique_events

def calculate_social_metrics(df):
    stats = {}

    # --- 0. DATA CUTOFF LOGIC (GLOBAL) ---
    # We slice the dataframe immediately so ALL subsequent calculations (Jerk, Politeness, etc.)
    # only see the data up to the moment of success.
    
    rx_raw = df["gt_robot_x"].values if "gt_robot_x" in df.columns else df["pos_x"].values
    ry_raw = df["gt_robot_y"].values if "gt_robot_y" in df.columns else df["pos_y"].values
    
    dist_to_goal = np.sqrt((rx_raw - GOAL_X)**2 + (ry_raw - GOAL_Y)**2)
    at_goal_indices = np.where(dist_to_goal < GOAL_TOLERANCE)[0]
    
    if len(at_goal_indices) > 0:
        first_success_idx = at_goal_indices[0]
        # TRUNCATE DATA HERE
        df = df.iloc[:first_success_idx + 1].copy()
        print(f"   -> Success detected! Truncating data to {len(df)} frames.")
    else:
        print("   -> Robot did not reach goal tolerance. Using full data.")

    # Update Duration based on the (potentially truncated) dataframe
    stats["duration"] = (df.index.max() - df.index.min()).total_seconds()

    # --- 1. Kinematics (On truncated data) ---
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds().fillna(0)
    df_kin = df[df["delta_t"] > 0.0001].copy()
    
    df_kin["accel"] = df_kin["lin_vel"].diff() / df_kin["delta_t"]
    df_kin["jerk"] = df_kin["accel"].diff() / df_kin["delta_t"]
    stats["smoothness_score"] = df_kin["jerk"].abs().rolling(10).mean().mean()

    # --- 2. Path & PIR (On truncated data) ---
    path_dist = np.sqrt(df["pos_x"].diff()**2 + df["pos_y"].diff()**2).sum()
    start_pos = np.array([df["pos_x"].iloc[0], df["pos_y"].iloc[0]])
    optimal_dist = np.linalg.norm(np.array([GOAL_X, GOAL_Y]) - start_pos)
    
    stats["path_length"] = path_dist
    stats["pir"] = path_dist / optimal_dist if optimal_dist > 0 else 0

    # --- 3. Social Metrics (On truncated data) ---
    rx = df["gt_robot_x"].values if "gt_robot_x" in df.columns else df["pos_x"].values
    ry = df["gt_robot_y"].values if "gt_robot_y" in df.columns else df["pos_y"].values
    
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    human_prefixes = [c.replace("_x", "") for c in human_x_cols]
    
    raw_events_metadata = [] 
    
    if human_prefixes:
        min_dists = []
        min_ttcs = []
        
        rvx = np.gradient(rx)
        rvy = np.gradient(ry)
        timestamps_numeric = df.index.astype(np.int64) / 1e9

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
            
            x_local = dx * np.cos(-h_yaw) - dy * np.sin(-h_yaw)
            y_local = dx * np.sin(-h_yaw) + dy * np.cos(-h_yaw)
            in_ellipse = ((x_local / ELLIPSE_A)**2 + (y_local / ELLIPSE_B)**2) <= 1
            
            vx_rel = rvx - hvx; vy_rel = rvy - hvy
            dot_prod = (dx * vx_rel) + (dy * vy_rel)
            speed_rel_towards = dot_prod / (dist + 0.001)
            
            ttc = np.full_like(dist, 10.0) 
            closing_mask = speed_rel_towards > 0.05 
            ttc[closing_mask] = dist[closing_mask] / speed_rel_towards[closing_mask]
            
            robot_speed = np.sqrt(rvx**2 + rvy**2)
            static_danger_mask = (dist < 0.5) & (robot_speed < 0.05)
            ttc[static_danger_mask] = 0.1 
            min_ttcs.append(np.min(ttc))

            violation_indices = np.where(in_ellipse)[0]
            
            for idx in violation_indices:
                # --- EXCLUSION ZONE BACKUP ---
                # Even with data truncated, we keep this to be safe for collisions RIGHT at the goal line.
                dist_from_exclusion = np.sqrt((rx[idx] - EXCLUSION_X)**2 + (ry[idx] - EXCLUSION_Y)**2)
                if dist_from_exclusion < EXCLUSION_RADIUS:
                    continue

                x_loc_val = x_local[idx]
                if x_loc_val > 0.5: type_str = "Frontal"
                elif x_loc_val < -0.5: type_str = "Rear"
                else: type_str = "Side"

                raw_events_metadata.append({
                    "human": h,
                    "type": type_str,
                    "frame_idx": idx,
                    "timestamp_val": timestamps_numeric[idx], 
                    "min_dist": dist[idx],
                    "rx": rx[idx], "ry": ry[idx],
                    "hx": hx[idx], "hy": hy[idx],
                    "rvx": rvx[idx], "rvy": rvy[idx],
                    "hvx": hvx[idx], "hvy": hvy[idx],
                    "h_yaw": h_yaw[idx]
                })

        df["nearest_human_dist"] = np.min(min_dists, axis=0)
        
        stats["min_human_dist"] = np.min(df["nearest_human_dist"])
        stats["min_ttc"] = np.min(min_ttcs) if min_ttcs else 10.0
        
        # MERGING
        filtered_events = filter_events_spatial_temporal(
            raw_events_metadata, 
            time_thresh=MERGE_TIME_THRESHOLD,
            dist_thresh=MERGE_DIST_THRESHOLD
        )
        
        filtered_events.sort(key=lambda x: x["min_dist"])
        
        stats["events"] = filtered_events
        stats["intrusion_count"] = len(filtered_events)
        stats["safety_margin"] = stats["min_human_dist"] - (ROBOT_RADIUS + HUMAN_RADIUS)
        
        if filtered_events:
            ev = filtered_events[0]
            dx = ev["rx"] - ev["hx"]; dy = ev["ry"] - ev["hy"]
            y_loc = dx * np.sin(-ev["h_yaw"]) + dy * np.cos(-ev["h_yaw"])
            stats["passing_preference"] = "Left" if y_loc > 0 else "Right"
        else:
            stats["passing_preference"] = "N/A"
        
        stats["has_social_data"] = True
    else:
        stats["min_human_dist"] = -1
        stats["has_social_data"] = False

    return df, stats

def save_csv_per_run(stats, filename, output_dir):
    csv_filename = os.path.splitext(os.path.basename(filename))[0] + ".csv"
    csv_path = os.path.join(output_dir, csv_filename)
    row = {
        "filename": filename,
        "duration": stats.get("duration", 0),
        "path_length": stats.get("path_length", 0),
        "pir": stats.get("pir", 0),
        "avg_jerk": stats.get("smoothness_score", 0),
        "min_human_dist": stats.get("min_human_dist", -1),
        "safety_margin": stats.get("safety_margin", -99),
        "min_ttc": stats.get("min_ttc", 999),
        "intrusion_count": stats.get("intrusion_count", 0),
        "passing_preference": stats.get("passing_preference", "N/A"),
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
    
    human_x_cols = [c for c in df.columns if "human" in c and c.endswith("_x")]
    for h_col in human_x_cols:
        h_prefix = h_col.replace("_x", "")
        valid_mask = df[h_col].abs() < 50
        ax1.plot(df.loc[valid_mask, f"{h_prefix}_x"].values, 
                 df.loc[valid_mask, f"{h_prefix}_y"].values, 
                 color="grey", alpha=0.3, linewidth=1)

    rx = df["gt_robot_x"].values if "gt_robot_x" in df.columns else df["pos_x"].values
    ry = df["gt_robot_y"].values if "gt_robot_y" in df.columns else df["pos_y"].values
    ax1.plot(rx, ry, color=COLOR_ROBOT, label="Robot Path", linewidth=2)
    ax1.plot(rx[0], ry[0], "go"); ax1.plot(rx[-1], ry[-1], "rx")
    
    if stats.get("events"):
        for i, event in enumerate(stats["events"]):
            deg = np.degrees(event["h_yaw"])
            ell = Ellipse((event["hx"], event["hy"]), width=ELLIPSE_A*2, height=ELLIPSE_B*2, 
                          angle=deg, edgecolor='red', facecolor='none', linewidth=2, linestyle='--')
            ax1.add_patch(ell)
            ax1.text(event["rx"], event["ry"], f"X{i+1}", color='red', fontweight='bold', fontsize=12)

    legend_elements = [
        Line2D([0], [0], color='grey', alpha=0.5, lw=2, label='Human Paths'),
        Line2D([0], [0], color='blue', lw=2, label='Robot Path'),
        Line2D([0], [0], color='red', marker='X', linestyle='None', markersize=10, label='Incidents'),
        Patch(edgecolor='red', facecolor='none', linestyle='--', label='Collision Zones')
    ]
    ax1.legend(handles=legend_elements, loc="upper right")
    ax1.grid(True, linestyle=':')

    # ==========================================================
    # 2. TOP MIDDLE: Safety Monitor
    # ==========================================================
    ax2 = fig.add_subplot(gs[0, 1])
    if stats["has_social_data"]:
        ax2.set_title("Safety Monitor")
        ax2.set_xlabel("Time (HH:MM:SS)")
        ax2.set_ylabel("Distance (m)", color='orange')
        
        l1 = ax2.plot(df.index.values, df["nearest_human_dist"].values, color="orange", label="Dist", linewidth=2)
        ax2.axhline(y=0.45, color='red', linestyle='--', alpha=0.5, label="Limit")
        ax2.tick_params(axis='y', labelcolor='orange')
        
        # Red Dots
        collisions = df[df["nearest_human_dist"] < 0.45]
        if not collisions.empty:
            ax2.plot(collisions.index.values, collisions["nearest_human_dist"].values, 
                     'r.', markersize=10)

        # TTC
        ax2_r = ax2.twinx()
        ax2_r.set_ylabel("TTC (s)", color='purple')
        ttc_raw = df["nearest_human_dist"] / (df["lin_vel"] + 0.01)
        ttc_raw = ttc_raw.clip(upper=10.0) 
        
        ttc_smooth = ttc_raw.rolling(window=5, center=True).median().fillna(ttc_raw)
        danger_mask = ttc_smooth < DANGER_TTC_LIMIT
        ax2_r.fill_between(df.index.values, 0, 10, where=danger_mask, 
                         color=COLOR_DANGER_FILL, alpha=0.5, transform=ax2_r.get_xaxis_transform(), label="Danger")
        
        ax2_r.set_ylim(0, 10); ax2_r.set_yticks([]) 
        lines = l1 + [Patch(color=COLOR_DANGER_FILL, alpha=0.5, label='Danger (<2s)')]
        ax2.legend(lines, [l.get_label() for l in lines], loc="upper right")
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
        
        x_zone = np.linspace(0, 10, 100)
        ax3.fill_between(x_zone, 0.1*x_zone, 2.0, color=COLOR_POLITE_ZONE, alpha=0.2, label="Polite")
        ax3.fill_between(x_zone, -0.1, 0.1*x_zone, color=COLOR_RUDE_ZONE, alpha=0.2, label="Rude")

        x = df["nearest_human_dist"].values; y = df["lin_vel"].values
        mask = ~np.isnan(x) & ~np.isnan(y)
        x_c = x[mask]; y_c = y[mask]
        
        if len(x_c) > 10:
            xl, xh = np.percentile(x_c, [5, 95])
            yl, yh = np.percentile(y_c, [5, 95])
            m_c = (x_c > xl) & (x_c < xh) & (y_c > yl) & (y_c < yh)
            xf, yf = x_c[m_c], y_c[m_c]
            
            # Transparent Blue
            ax3.scatter(xf, yf, color='tab:blue', alpha=0.3, s=15)
            
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
    # 4. BOTTOM RIGHT: Zoom-Ins (Reverted to Transparent Circles)
    # ==========================================================
    ax_container = fig.add_subplot(gs[1, 1])
    ax_container.set_xticks([]); ax_container.set_yticks([])
    ax_container.set_title("COLLISION INSTANCE DETAILS AS EVALUATION METRIC", fontsize=11, pad=20) 
    
    # Thin Border
    for spine in ax_container.spines.values():
        spine.set_linewidth(1.0); spine.set_edgecolor('black')

    custom_lines = [Line2D([0], [0], color='red', lw=2, label='Human'),
                    Line2D([0], [0], color='blue', lw=2, label='Robot'),
                    Patch(color='red', alpha=0.1, label='Zone')]
    ax_container.legend(handles=custom_lines, loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=3, frameon=False, fontsize='small')

    events = stats.get("events", [])
    num_show = min(len(events), 4)
    
    if num_show > 0:
        gs_inner = gridspec.GridSpecFromSubplotSpec(2, 2, subplot_spec=gs[1, 1], wspace=0.1, hspace=0.3)
        
        for i in range(num_show):
            ax_sub = fig.add_subplot(gs_inner[i])
            ev = events[i]
            
            # Label above
            ax_sub.text(ev["hx"], ev["hy"] + 1.2, f"X{i+1}: {ev['type']} ({ev['min_dist']:.2f}m)", 
                        ha='center', va='bottom', fontsize=9, color='black')
            
            ax_sub.set_xlim(ev["rx"] - 2.0, ev["rx"] + 2.0)
            ax_sub.set_ylim(ev["ry"] - 2.0, ev["ry"] + 2.0)
            ax_sub.axis('off') 
            
            deg = np.degrees(ev["h_yaw"])
            ell = Ellipse((ev["hx"], ev["hy"]), width=ELLIPSE_A*2, height=ELLIPSE_B*2, 
                          angle=deg, color='r', alpha=0.2)
            ax_sub.add_patch(ell)
            
            # --- TRANSPARENT CIRCLES & ARROWS ---
            # Human
            ax_sub.add_patch(Circle((ev["hx"], ev["hy"]), ROBOT_RADIUS, color='r', fill=True, alpha=0.3))
            # ax_sub.add_patch(Circle((ev["hx"], ev["hy"]), 0.15, color='r', fill=True, alpha=0.3))
            ax_sub.arrow(ev["hx"], ev["hy"], ev["hvx"], ev["hvy"], head_width=0.2, color='r', length_includes_head=True)
            # Robot
            ax_sub.add_patch(Circle((ev["rx"], ev["ry"]), ROBOT_RADIUS, color='b', fill=True, alpha=0.3))
            ax_sub.arrow(ev["rx"], ev["ry"], ev["rvx"], ev["rvy"], head_width=0.2, color='b', length_includes_head=True)
            
    else:
        ax_container.text(0.5, 0.5, "Safe Run\nNo Collisions Detected", ha='center', va='center')

    # ==========================================================
    # 5. RIGHT SIDEBAR: Text Report
    # ==========================================================
    ax5 = fig.add_subplot(gs[:, 2])
    ax5.axis("off")
    
    lines = [
        "--- RUN SUMMARY ---",
        f"Time:       {stats.get('duration', 0):.2f} s",
        f"Path Len:   {stats['path_length']:.2f} m",
        # Renamed Smoothness -> Avg Jerk
        f"Avg Jerk:   {stats['smoothness_score']:.2f} m/s^3",
        "            (< 5 is smooth)",
        f"PIR:        {stats['pir']:.2f} (1.0=Opt)",
        "",
        "--- SOCIAL ANALYSIS ---",
    ]
    
    if stats["has_social_data"]:
        margin = stats['safety_margin']
        lines.append(f"Min Dist:   {stats['min_human_dist']:.2f} m")
        
        if margin < 0: lines.append("STATUS:     COLLISION")
        else: lines.append(f"Margin:     {margin:.2f} m")
            
        lines.append(f"Min TTC:    {stats['min_ttc']:.2f} s")
        lines.append(f"Violations: {stats['intrusion_count']}")
        lines.append("")
        lines.append(f"Pass Side:  {stats['passing_preference']}")
        lines.append(f"Politeness: {slope_status}")
        lines.append(f"Slope:      {slope_val:.3f}")
        
        status = "FAIL" if (stats['min_human_dist'] < 0.45 or stats['min_ttc'] < 0.5) else "PASS"
        lines.append("")
        lines.append(f"RESULT:     {status}")
        
        if num_show > 0:
            lines.append("")
            lines.append("--- EVENTS ---")
            for i in range(num_show):
                ev = events[i]
                lines.append(f"X{i+1}: {ev['type']} ({ev['min_dist']:.2f}m)")

    else:
        lines.append("No Human Data")

    # Force Black Text
    ax5.text(0.05, 0.95, "\n\n".join(lines), transform=ax5.transAxes, 
             fontsize=14, family="monospace", va="top", color='black')

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