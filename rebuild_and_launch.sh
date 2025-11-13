#!/usr/bin/env bash

set -euo pipefail

LAYOUT_FILE="install/.colcon_install_layout"
COLCON_LAYOUT=${COLCON_LAYOUT:-}
if [ -z "${COLCON_LAYOUT}" ] && [ -f "${LAYOUT_FILE}" ]; then
    COLCON_LAYOUT=$(<"${LAYOUT_FILE}")
fi

COLCON_ARGS=()
case "${COLCON_LAYOUT:-isolated}" in
    merged)
        COLCON_ARGS+=(--merge-install)
        ;;
    isolated)
        ;;
    *)
        echo "Unknown install layout '${COLCON_LAYOUT}', defaulting to isolated." >&2
        ;;
esac

colcon build "${COLCON_ARGS[@]}"
set +u
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source install/setup.bash
set -u
ros2 launch sm_mppi_planner gazebo_relay_node_all_simulations.launch.py scenario_id:=1
# ros2 launch sm_mppi_planner scenario_1_gauntlet.launch.py
