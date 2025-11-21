#!/bin/bash

BAG_PATH=$1

if [ -z "$BAG_PATH" ]; then
    echo "Usage: ./run_repair.sh <path_to_bag_folder>"
    exit 1
fi

BAG_NAME=$(basename "$BAG_PATH")
PARENT_DIR=$(dirname "$BAG_PATH")
OUTPUT_FILE="$PARENT_DIR/$BAG_NAME.parquet"

echo "-------------------------------------------------"
echo "PROCESSING: $BAG_NAME"
echo "OUTPUT:     $OUTPUT_FILE"
echo "-------------------------------------------------"

source /opt/ros/humble/setup.bash

echo "[INFO] Starting Logger Node..."
# Path relative to PROJECT ROOT
python3 src/sm_mppi_planner/sm_mppi_planner/logger_node.py --ros-args -p output_filename:="$OUTPUT_FILE" &
LOGGER_PID=$!

sleep 3

echo "[INFO] Playing Bag at 5x speed..."
ros2 bag play "$BAG_PATH" -r 5.0 --clock

echo ""
echo "[INFO] Bag finished. Stopping Logger..."
kill -SIGINT $LOGGER_PID
wait $LOGGER_PID

echo "[SUCCESS] Parquet file saved to: $OUTPUT_FILE"
