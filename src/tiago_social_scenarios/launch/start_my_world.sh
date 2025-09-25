#!/usr/bin/env bash
set -e

# 0) Launch sim (new terminal recommended)
ros2 launch tiago_gazebo tiago_gazebo.launch.py navigation:=True is_public_sim:=True world_name:=my_world &
LAUNCH_PID=$!

# 1) Wait for nodes to appear
sleep 8
# (retry a few times if needed)
for i in {1..10}; do
  ros2 node list | grep -q /map_server && break || sleep 1
done
for i in {1..10}; do
  ros2 node list | grep -q /amcl && break || sleep 1
done

# 2) Map server -> your map (configure -> activate)
ros2 param set /map_server yaml_filename /home/wim/Documents/social_momentum_venv/social_momentum_MPPI/src/tiago_social_scenarios/maps/hallway_map.yaml || true
ros2 lifecycle set /map_server configure || true
ros2 lifecycle set /map_server activate || true

# 3) AMCL -> correct laser/frames (configure -> activate)
ros2 param set /amcl scan_topic /scan_raw || true
ros2 param set /amcl base_frame_id base_footprint || true
ros2 param set /amcl odom_frame_id odom || true
ros2 param set /amcl global_frame_id map || true
ros2 lifecycle set /amcl configure || true
ros2 lifecycle set /amcl activate || true

# 4) Initial pose (AMCL must be active)
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header:{frame_id: map}, pose:{pose:{position:{x:0.0,y:0.0,z:0.0}, orientation:{z:0.0,w:1.0}}, covariance:[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}}"

# 5) Quick sanity prints
ros2 service call /map_server/map nav2_msgs/srv/GetMap "{}"
ros2 topic info /scan_raw
echo "Set RViz Fixed Frame=map, add Map(/map) and LaserScan(/scan_raw)."
wait $LAUNCH_PID