#!/bin/bash

echo "--- Navigating to workspace directory ---"
cd "$(dirname "$0")"

echo "--- Cleaning old build, install, and log files ---"
rm -rf build install log

echo "--- Building the workspace with colcon ---"
colcon build --symlink-install
if [ $? -ne 0 ]; then
    echo "COLCON BUILD FAILED. Aborting."
    exit 1
fi

echo "--- Sourcing the new installation ---"
source /opt/ros/humble/setup.bash
source /home/wim/tiago_public_ws/install/setup.bash
source install/setup.bash

echo "--- Launching the simulation ---"

# THIS IS THE FIX:
# Launch the SINGLE, CORRECT master launch file that starts everything.
ros2 launch tiago_social_scenarios start_nav2_scenario_1.launch.py