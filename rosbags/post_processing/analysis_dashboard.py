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
        # ... (error handling remains the same)
        sys.exit(1)
        
    print(f"Loading data from {parquet_file}...")
    df = pd.read_parquet(parquet_file, engine="pyarrow")
    
    # --- FIX THE COLUMN NAME HERE ---
    # Replace "timestamp" with the actual column name found in your file
    ACTUAL_TIME_COLUMN = "timestamp" # <-- CHANGE THIS if necessary!
    
    if ACTUAL_TIME_COLUMN not in df.columns:
        # Provide a better error message if the column is still missing
        raise KeyError(
            f"Expected column '{ACTUAL_TIME_COLUMN}' not found. "
            f"Available columns are: {list(df.columns)}"
        )
        
    df.set_index(ACTUAL_TIME_COLUMN, inplace=True)
    df.sort_index(inplace=True)
    
    print(f"Loaded {len(df)} data points.")
    return df

def calculate_metrics(df):
    """
    Performs all advanced metric calculations.
    """
    print("Calculating metrics...")
    
    # --- 1. Calculate Time-Aware Derivatives (Accel, Jerk) ---
    # df.index is a DatetimeIndex
    # .diff() computes the difference between consecutive timestamps (Timedelta)
    # .dt.total_seconds() converts the Timedelta to a float number of seconds
    df["delta_t"] = df.index.to_series().diff().dt.total_seconds()
    df["acceleration_m_s2"] = df["lin_vel"].diff() / df["delta_t"]
    df["jerk_m_s3"] = df["acceleration_m_s2"].diff() / df["delta_t"]

    # --- 2. Calculate Aggregate KPIs ---
    total_duration = df.index.max() - df.index.min()
    joint_vel_cols = [col for col in df.columns if col.endswith("_vel")]
    avg_joint_speeds = df[joint_vel_cols].abs().mean()
    
    # --- 3. Calculate Total Energy (REMOVED) ---
    
    # --- 4. Clean and Store Results ---
    
    kpi_summary = {
        "total_duration": total_duration,
        "avg_joint_speeds": avg_joint_speeds,
    }
    
    return df, kpi_summary

# --- MODIFIED: Added 'output_dir' instead of 'parquet_file' to specify save location ---
def create_dashboard(df, kpi_summary, parquet_file, output_dir, show_plot=True):
    """
    Generates a 4-panel dashboard with Matplotlib and saves it to the specified output directory.
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
    ax1.axis("equal") # Ensure path scaling is accurate

    # --- Plot 2: Kinematics (Accel & Jerk) ---
    ax2 = axs[0, 1]
    ax2_twin = ax2.twinx()
    
    # Time is on the index, using df.index.values for x-axis
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
    # Combine legends from both axes
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
    # Keep only the date/time part of the timedelta for display
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
    
    # --- Finalize and Save ---
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    # Get the filename without the extension (e.g., "metrics_data_...")
    file_name_no_ext = os.path.splitext(os.path.basename(parquet_file))[0]
    
    # Join the output directory, the unique name, and the .png extension
    output_image = os.path.join(output_dir, file_name_no_ext + ".png")
    
    plt.savefig(output_image)
    print(f"Dashboard saved to {output_image}")
    
    if show_plot:
        plt.show()
    else:
        plt.close(fig)

# --- MODIFIED: Added 'output_dir' for file saving ---
def process_file(parquet_file, output_dir, show_plot):
    df = load_data(parquet_file)
    df_metrics, kpi_summary = calculate_metrics(df)
    create_dashboard(df_metrics, kpi_summary, parquet_file, output_dir, show_plot=show_plot)

# --- MODIFIED: The main logic is completely overhauled to handle folder mirroring ---
def main():
    if len(sys.argv) < 2:
        print("Error: Please provide the path to the data directory (e.g., 'rosbags/raw').")
        print("Usage: python3 analysis_dashboard.py <path_to_data_folder>")
        sys.exit(1)
        
    # The user provides the 'raw' directory (or a subdirectory inside it)
    target_path = os.path.abspath(sys.argv[1])
    
    if not os.path.isdir(target_path):
        print(f"Error: Target path is not a directory: {target_path}")
        sys.exit(1)
        
    # 1. Determine the root directory structure (e.g., 'rosbags/post_processing')
    
    # Get the directory where this script is located (rosbags/post_processing/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Assuming the script is in 'rosbags/post_processing', its parent is 'rosbags'
    rosbags_dir = os.path.abspath(os.path.join(script_dir, os.pardir))
    
    # Define the 'raw' and 'post_processing' directories based on the script's location
    post_processing_root = script_dir
    raw_root = os.path.join(rosbags_dir, "raw")
    
    # 2. Check if the provided path is the 'raw' root or a subdirectory
    if target_path == raw_root:
        # If the user points to 'rosbags/raw', we process all subdirectories inside it
        source_dirs = [
            os.path.join(raw_root, name) 
            for name in os.listdir(raw_root)
            if os.path.isdir(os.path.join(raw_root, name))
        ]
    elif os.path.dirname(target_path) == raw_root:
        # If the user points to 'rosbags/raw/SUBDIR', we process only that one
        source_dirs = [target_path]
    else:
        print(f"Error: Target path '{target_path}' does not seem to be inside '{raw_root}'.")
        print("Please point to the 'raw' directory or one of its immediate subdirectories.")
        sys.exit(1)

    if not source_dirs:
        print(f"No subdirectories found in {raw_root} to process.")
        sys.exit(1)

    print(f"Found {len(source_dirs)} source folders to process.")
    
    for source_folder in source_dirs:
        folder_name = os.path.basename(source_folder)
        
        # Define the mirrored output folder (e.g., 'rosbags/post_processing/REPAIRED_NAV2')
        output_folder = os.path.join(post_processing_root, folder_name)
        
        # Create the output directory if it doesn't exist
        os.makedirs(output_folder, exist_ok=True)
        
        print(f"\n--- Processing folder: {folder_name} ---")
        
        # Find all .parquet files in the current source folder
        parquet_files = [
            os.path.join(source_folder, fname)
            for fname in sorted(os.listdir(source_folder))
            if fname.lower().endswith(".parquet")
        ]

        if not parquet_files:
            print(f"No .parquet files found in directory: {source_folder}")
            continue

        print(f"Found {len(parquet_files)} parquet files. Saving dashboards to {output_folder} ...")
        
        # Process all files in the batch
        for file_path in parquet_files:
            print(f"  > Processing {os.path.basename(file_path)} ...")
            # The dashboard is saved to the 'output_folder'
            process_file(file_path, output_folder, show_plot=False)

if __name__ == "__main__":
    main()