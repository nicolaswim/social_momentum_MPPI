#!/usr/bin/env bash

set -euo pipefail

# This script ONLY sources and launches.
# Run 'colcon build' manually before you start the batch.

set +u
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source install/setup.bash
set -u

# We use $1 so the batch script can pass the scenario_id
# Default to 1 if it's not provided
SCENARIO_ID="${1:-1}"
echo "--- Launching scenario_id: $SCENARIO_ID ---"
ros2 launch sm_mppi_planner gazebo_relay_node_all_simulations.launch.py scenario_id:="$SCENARIO_ID"