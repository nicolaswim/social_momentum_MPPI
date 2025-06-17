#!/bin/bash

# This script will stop immediately if any command fails.
set -e

echo "--- Navigating to workspace directory ---"
cd ~/Documents/social_momentum_venv/social_momentum_MPPI

echo "--- Cleaning old build, install, and log files ---"
rm -rf build/ install/ log/

echo "--- Building the workspace with colcon ---"
colcon build

echo "--- Sourcing the new installation ---"
source install/setup.bash

echo "--- Launching the simulation ---"
ros2 launch sm_mppi_planner sm_paramtrized_version.launch.py