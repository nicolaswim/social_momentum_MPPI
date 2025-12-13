import os
import glob
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# ================= CONFIGURATION =================
NAV2_DIR = "./NAV2"
SM_DIR = "./social_momentum"
OUTPUT_PLOT = "Scientific_Comparison_Dashboard.png"
OUTPUT_CSV = "comparison_summary.csv"
# =================================================

def parse_scenario_id(filename, method):
    """
    Parses the filename to extract the scenario number.
    """
    base = os.path.basename(filename)
    parts = base.split('_')
    
    try:
        if method == "Nav2":
            # Format: scenario_backup_1_... -> Index 2 is the ID
            return parts[2]
        elif method == "Social Momentum":
            # Format: metrics_data_1_... -> Index 2 is the ID
            return parts[2]
    except IndexError:
        return "Unknown"
    return "Unknown"

def load_and_clean_data():
    """
    Crawls directories, cleans data, renames Scenario 4 -> 3, and sorts.
    """
    all_data = []

    # 1. Load Nav2
    nav2_files = glob.glob(os.path.join(NAV2_DIR, "*.csv"))
    print(f"Found {len(nav2_files)} Nav2 runs.")
    for f in nav2_files:
        try:
            df = pd.read_csv(f)
            df['Algorithm'] = 'Nav2'
            df['Scenario'] = parse_scenario_id(f, 'Nav2')
            all_data.append(df)
        except Exception as e:
            print(f"Skipping {f}: {e}")

    # 2. Load Social Momentum
    sm_files = glob.glob(os.path.join(SM_DIR, "*.csv"))
    print(f"Found {len(sm_files)} Social Momentum runs.")
    for f in sm_files:
        try:
            df = pd.read_csv(f)
            df['Algorithm'] = 'Social Momentum'
            df['Scenario'] = parse_scenario_id(f, 'Social Momentum')
            all_data.append(df)
        except Exception as e:
            print(f"Skipping {f}: {e}")

    if not all_data:
        print("No data found.")
        return pd.DataFrame()

    # 3. Merge
    master_df = pd.concat(all_data, ignore_index=True)

    # 4. Filter for Scenarios 1, 2, 4
    master_df = master_df[master_df['Scenario'].isin(['1', '2', '4'])]

    # 5. RENAME Scenario 4 -> Scenario 3
    master_df['Scenario'] = master_df['Scenario'].replace('4', '3')

    # 6. Force specific sort order (1 -> 2 -> 3)
    master_df['Scenario_Int'] = master_df['Scenario'].astype(int)
    master_df = master_df.sort_values(by='Scenario_Int')

    return master_df

def save_statistics(df):
    """
    Calculates mean and std deviation and saves to CSV.
    """
    # Select numeric columns relevant to the study
    numeric_cols = ['duration', 'avg_jerk', 'min_human_dist', 'pir']
    
    # Group by Algorithm and Scenario
    summary = df.groupby(['Algorithm', 'Scenario'])[numeric_cols].agg(['mean', 'std'])
    
    # Round for cleanliness
    summary = summary.round(3)
    
    # Save
    summary.to_csv(OUTPUT_CSV)
    print(f"\nStatistics saved to: {OUTPUT_CSV}")
    print("-" * 30)
    print(summary)
    print("-" * 30)

def generate_scientific_dashboard(df):
    """
    Generates the strict scientific dashboard (Boxplots only).
    """
    # Visual Setup
    sns.set_theme(style="whitegrid", font_scale=1.1)
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    fig.suptitle('Nav2 vs. Social Momentum: Performance Metrics', fontsize=22, weight='bold')

    # Palette
    palette = {"Nav2": "#4c72b0", "Social Momentum": "#dd8452"}
    
    # Plot Arguments
    plot_args = {
        'x': 'Scenario',
        'hue': 'Algorithm',
        'palette': palette,
        'linewidth': 1.5,
        'fliersize': 4, # Outlier markers
        'width': 0.7
    }

    # --- PLOT 1: DURATION ---
    sns.boxplot(data=df, y='duration', ax=axes[0, 0], **plot_args)
    axes[0, 0].set_title('Time to Goal', fontsize=14, weight='bold')
    axes[0, 0].set_ylabel('Duration (s)')
    axes[0, 0].legend(loc='upper left')

    # --- PLOT 2: JERK ---
    sns.boxplot(data=df, y='avg_jerk', ax=axes[0, 1], **plot_args)
    axes[0, 1].set_title('Average Jerk (Smoothness)', fontsize=14, weight='bold')
    axes[0, 1].set_ylabel('Jerk ($m/s^3$)')
    axes[0, 1].legend(loc='upper right')

    # --- PLOT 3: DISTANCE ---
    sns.boxplot(data=df, y='min_human_dist', ax=axes[1, 0], **plot_args)
    axes[1, 0].set_title('Minimum Distance to Human', fontsize=14, weight='bold')
    axes[1, 0].set_ylabel('Distance (m)')
    # CHANGED LOCATION HERE TO UPPER RIGHT
    axes[1, 0].legend(loc='upper right')
    
    # Add Intimate Zone Reference
    axes[1, 0].axhline(0.45, color='red', linestyle='--', linewidth=1.5, alpha=0.7)
    axes[1, 0].text(axes[1, 0].get_xlim()[0] + 0.1, 0.40, "Intimate Zone", color='red', fontsize=10)

    # --- PLOT 4: PIR ---
    sns.boxplot(data=df, y='pir', ax=axes[1, 1], **plot_args)
    axes[1, 1].set_title('Path Irregularity Ratio (PIR)', fontsize=14, weight='bold')
    axes[1, 1].set_ylabel('PIR (1.0 = Optimal)')
    axes[1, 1].legend(loc='upper right')
    
    # Add Optimal Path Reference
    axes[1, 1].axhline(1.0, color='green', linestyle='--', linewidth=1.5, alpha=0.7)

    # Layout adjustment and Save
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(OUTPUT_PLOT, dpi=300)
    print(f"\nDashboard saved to: {OUTPUT_PLOT}")
    plt.close()

if __name__ == "__main__":
    print("--- Starting Scientific Analysis ---")
    df = load_and_clean_data()
    
    if not df.empty:
        # 1. Output Statistics to CSV
        save_statistics(df)
        
        # 2. Generate Plot
        generate_scientific_dashboard(df)
        
        print("\nProcess Complete.")
    else:
        print("Error: No data found.")