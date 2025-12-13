# 1. Define the problematic input path
BAG_TO_FIX="/home/wim/Documents/social_momentum_venv/social_momentum_MPPI/rosbags/raw/NAV2/scenario_backup_1_20251120_234406"

# 2. Define the temporary, clean location
TEMP_BAG_DIR="/tmp/temp_bag_for_repair"

# 3. Clean up any previous attempts and copy the directory (Force copy)
rm -rf "$TEMP_BAG_DIR"
cp -r "$BAG_TO_FIX" "$TEMP_BAG_DIR"

echo "--- Attempting to REPAIR the bag by converting it to a new location ---"
BAG_TO_CONVERT="/tmp/temp_bag_for_conversion"
rm -rf "$BAG_TO_CONVERT"

source /opt/ros/humble/setup.bash

# CORRECTED COMMAND
ros2 bag convert -i "$TEMP_BAG_DIR" -o "$BAG_TO_CONVERT"

echo "--- Bag conversion/repair completed. New bag is at $BAG_TO_CONVERT ---"

# 5. Define the output path for the permanent Parquet file
PARQUET_OUTPUT_FILE="/home/wim/Documents/social_momentum_venv/social_momentum_MPPI/rosbags/raw/PROCESSED/scenario_1_metrics_final.parquet"

# 6. Execute the conversion script using the newly converted (repaired) bag
python3 src/sm_mppi_planner/sm_mppi_planner/bag_to_parquet_converter.py \
    "$BAG_TO_CONVERT" \
    "$PARQUET_OUTPUT_FILE"

# 7. Clean up the temporary directories
rm -rf "$TEMP_BAG_DIR"
rm -rf "$BAG_TO_CONVERT"
