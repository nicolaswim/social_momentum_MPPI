#!/usr/bin/env python3

## Component 2: Offline Analysis and Visualization
##
## This script is run *after* the ROS run is complete.
## Reads the 'robot_run_data.parquet' file.
##
## It performs time-aware calculations for:
## 1. Calculates time-deltas (delta_t) for non-uniform data.
## 2. Calculates acceleration and jerk (dv/dt, da/dt).
## 3. Calculates aggregate KPIs (duration, avg speed).
##
## Finally, it generates a multi-panel Matplotlib dashboard.
##

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def load_data(parquet_file):
    """
    Loads the Parquet file and sets up the time-series index.
    """
    if not os.path.exists(parquet_file):
        print(f"Error: File not found: {parquet_file}")
        print("Please run the logger_node.py first.")
        sys.exit(1)
        
    print(f"Loading data from {parquet_file}...")
    df = pd.read_parquet(parquet_file, engine="pyarrow")
    
    df.set_index("timestamp", inplace=True)
    df.sort_index(inplace=True)
    
    print(f"Loaded {len(df)} data points.")
    return df

def calculate_metrics(df):
    """
    Performs all advanced metric calculations.
    """
    print("Calculating metrics...")
    
    # --- 1. Calculate Time-Aware Derivatives (Accel, Jerk) ---
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds()
    df["acceleration_m_s2"] = df["lin_vel"].diff() / df["delta_t"]
    df["jerk_m_s3"] = df["acceleration_m_s2"].diff() / df["delta_t"]

    # --- 2. Calculate Aggregate KPIs ---
    total_duration = df.index.max() - df.index.min()
    joint_vel_cols = [col for col in df.columns if col.endswith("_vel")]
    avg_joint_speeds = df[joint_vel_cols].abs().mean()
    
    # --- 3. Calculate Total Energy (REMOVED) ---
    
    # --- 4. Clean and Store Results ---
    # df.dropna(inplace=True) # Removed this line
    
    # --- MODIFIED: 'total_energy_wh' removed ---
    kpi_summary = {
        "total_duration": total_duration,
        "avg_joint_speeds": avg_joint_speeds,
    }
    
    return df, kpi_summary

# --- MODIFIED: Added 'parquet_file' as an argument ---
def create_dashboard(df, kpi_summary, parquet_file, show_plot=True):
    """
    Generates a 4-panel dashboard with Matplotlib.
    """
    print("Generating dashboard...")
    
    # Create a 2x2 grid for the plots
    fig, axs = plt.subplots(2, 2, figsize=(18, 12))
    fig.suptitle("Robot Performance Analysis Dashboard", fontsize=20)

    # --- Plot 1: 2D Robot Path ---
    ax1 = axs[0, 0]
    ax1.plot(df["pos_x"].values, df["pos_y"].values, color="blue", label="Path")
    ax1.plot(
        df["pos_x"].iloc[0],
        df["pos_y"].iloc[0],
        "go",
        markersize=10,
        label="Start",
    )
    ax1.plot(
        df["pos_x"].iloc[-1],
        df["pos_y"].iloc[-1],
        "rs",
        markersize=10,
        label="End",
    )
    ax1.set_title("Robot 2D Path")
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.legend()
    ax1.grid(True)
    ax1.axis("equal")

    # --- Plot 2: Kinematics (Accel & Jerk) ---
    ax2 = axs[0, 1]
    ax2_twin = ax2.twinx()
    
    (p1,) = ax2.plot(
        df.index.values,
        df["acceleration_m_s2"].values,
        color="green",
        alpha=0.8,
        label="Acceleration",
    )
    (p2,) = ax2_twin.plot(
        df.index.values, 
        df["jerk_m_s3"].values, 
        color="purple", 
        alpha=0.6, 
        label="Jerk"
    )
    
    ax2.set_title("Kinematic Analysis")
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Acceleration ($m/s^2$)", color="green")
    ax2_twin.set_ylabel("Jerk ($m/s^3$)", color="purple")
    ax2.legend(handles=[p1, p2], loc="upper right")
    ax2.grid(True)

    # --- Plot 3: Power Consumption (REMOVED) ---
    axs[1, 0].axis("off") 
    axs[1, 0].set_title("Power (Removed)")


    # --- Plot 4: KPI Summary ---
    ax4 = axs[1, 1]
    ax4.axis("off") # Hide axis lines and ticks
    ax4.set_title("Aggregate KPIs")
    
    # Format text
    duration_str = str(kpi_summary["total_duration"]).split(".")[0]
    
    text_lines = [
        "--- RUN SUMMARY ---",
        f"Total Duration:  {duration_str}",
        "",
        "--- AVG. JOINT SPEEDS (rad/s) ---"
    ]
    
    # Add joint speeds
    for joint, speed in kpi_summary["avg_joint_speeds"].items():
        text_lines.append(f"  - {joint}: {speed:.3f}")

    # Display text
    ax4.text(
        0.05,
        0.95,
        "\n".join(text_lines),
        transform=ax4.transAxes,
        fontsize=12,
        verticalalignment="top",
        fontfamily="monospace",
    )
    
    # --- Finalize and Show ---
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    # --- MODIFIED: Generate unique name and save to script's directory ---
    
    # Get the directory where this script is located (rosbags/post_processing/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Get the base filename from the parquet file (e.g., "metrics_data_... .parquet")
    base_filename = os.path.basename(parquet_file)
    
    # Get the filename without the extension (e.g., "metrics_data_...")
    file_name_no_ext = os.path.splitext(base_filename)[0]
    
    # Join the script's directory, the unique name, and the .png extension
    # This will save to "rosbags/post_processing/metrics_data_... .png"
    output_image = os.path.join(script_dir, file_name_no_ext + ".png")
    
    plt.savefig(output_image)
    print(f"Dashboard saved to {output_image}")
    
    if show_plot:
        plt.show()
    else:
        plt.close(fig)

def process_file(parquet_file, show_plot):
    df = load_data(parquet_file)
    df_metrics, kpi_summary = calculate_metrics(df)
    create_dashboard(df_metrics, kpi_summary, parquet_file, show_plot=show_plot)

def main():
    if len(sys.argv) > 1:
        target_path = sys.argv[1]
    else:
        print("Error: Please provide the path to the .parquet file.")
        print("Usage: python3 analysis_dashboard.py <path_to_data.parquet>")
        sys.exit(1)
        
    target_path = os.path.abspath(target_path)

    if os.path.isdir(target_path):
        parquet_files = [
            os.path.join(target_path, fname)
            for fname in sorted(os.listdir(target_path))
            if fname.lower().endswith(".parquet")
        ]

        if not parquet_files:
            print(f"No .parquet files found in directory: {target_path}")
            sys.exit(1)

        print(f"Found {len(parquet_files)} parquet files in {target_path}. Batch processing...")
        for file_path in parquet_files:
            print(f"\nProcessing {file_path} ...")
            process_file(file_path, show_plot=False)
    else:
        process_file(target_path, show_plot=True)

if __name__ == "__main__":
    main()
