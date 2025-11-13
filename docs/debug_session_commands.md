## Debug Session Cheat Sheet

Use these terminal-ready blocks whenever you need to reproduce a slipping run and capture diagnostics. Each block can be copied verbatim into a new terminal window/tab.

### Terminal A – Simulator
```bash
cd ~/Documents/social_momentum_venv/social_momentum_MPPI
./run_batch.sh
```
Leave this running; it rebuilds/launches the scenario every two minutes.

### Terminal B – Data Capture
```bash
cd ~/Documents/social_momentum_venv/social_momentum_MPPI
./scripts/capture_debug_data.sh
```
The script now records `/tf`, `/tf_static`, `/clock`, `/mobile_base_controller/odom`, `/ground_truth_odom`, `/model_states`, runs TF monitors for `odom→base_link` and `base_footprint→base_link`, and stores a snapshot of `ros2 topic list` / `ros2 node list`. All artifacts land under `rosbags/debug_sessions/<timestamp>/`.

### Terminal C – Quick Inspection (optional)
```bash
cd ~/Documents/social_momentum_venv/social_momentum_MPPI
source /opt/ros/humble/setup.bash
ros2 bag info rosbags/debug_sessions/<latest_session>/bag
```
Replace `<latest_session>` with the folder created by Terminal B to confirm the bag contains the expected topics before analyzing further.

All generated logs (topic list, node list, TF monitors, rosbag recorder output) are plain text files inside the session directory, so you can review or share them later without rerunning the sim.
