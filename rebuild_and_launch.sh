# #!/bin/bash

# # This script will stop immediately if any command fails.
# set -e

# echo "--- Navigating to workspace directory ---"
# cd ~/Documents/social_momentum_venv/social_momentum_MPPI

# echo "--- Cleaning old build, install, and log files ---"
# rm -rf build/ install/ log/

# echo "--- Building the workspace with colcon ---"
# colcon build

# echo "--- Sourcing the new installation ---"
# source install/setup.bash

# echo "--- Launching the simulation ---"
# # ros2 launch sm_mppi_planner sm_paramtrized_version.launch.py
# # ros2 launch sm_mppi_planner view_hallway.launch.py
# # ros2 launch sm_mppi_planner first_scenario_test.launch.py

# # ros2 launch sm_mppi_planner scenario_1_gauntlet.launch.py
# # ros2 launch sm_mppi_planner scenario_2_crossroads.launch.py
# # ros2 launch sm_mppi_planner scenario_3_cluster.launch.py
# # ros2 launch sm_mppi_planner scenario_4_emergency.launch.py 
# # ros2 launch sm_mppi_planner scenario_5_groups.launch.py

# # ros2 launch sm_mppi_planner testing_wheelchairs.launch.py
# # ros2 launch tiago_social_scenarios scenario_1_gazebo_nav.launch.py
# ros2 launch tiago_social_scenarios my_world_nav.launch.py


# In your workspace root
colcon build 
source install/setup.bash
# ros2 launch tiago_social_scenarios start_hallway_nav.launch.py world_name:=scenario_1
ros2 launch sm_mppi_planner joe_test.launch.py