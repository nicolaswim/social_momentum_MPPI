colcon build 
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source install/setup.bash
ros2 launch sm_mppi_planner gazebo_relay_node_all_simulations.launch.py scenario_id:=1