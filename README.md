Of course. Based on the files and information you've provided—including your thesis context, the foundational scientific paper, and the Python source code for your ROS 2 package—I have generated a comprehensive `README.md` file for your project.

This README explains the purpose of the project, its theoretical underpinnings, the system architecture, and how to install and run it.

---

# SM-MPPI: A Socially-Aware Navigation Planner for ROS 2

This repository contains the ROS 2 implementation of a socially-aware local planner that combines the **Social Momentum** framework with **Model Predictive Path Integral (MPPI)** control. The planner is designed to enable a mobile robot to navigate legibly, safely, and cooperatively in dynamic environments shared with humans.

This package was developed as the primary case study for the Master's Thesis *"Value sensitive design in Engineering"* (Landuyt, 2024) at Delft University of Technology, serving as a practical application for bridging ethical design principles with robotics engineering.

## Core Concepts

The planner's intelligence stems from a novel cost function that integrates social awareness into the well-established MPPI control framework.

### 1. Social Momentum (SM)
[cite_start]The core social navigation logic is based on the paper **"Social Momentum: Design and Evaluation of a Framework for Socially Competent Robot Navigation"**[cite: 280]. The key insight is to model the abstract concept of a "passing agreement" between the robot and a human using a physics-inspired metric: **pairwise angular momentum**.

* When the robot and a human are about to pass each other, the direction (sign) of their collective angular momentum indicates a preference for passing on the right or left.
* The planner generates motion that actively **reinforces and increases** the magnitude of this momentum, making the robot's intention to yield or pass on a specific side clear and unambiguous. This "intent-expressive" or "legible" motion allows humans to predict the robot's path, fostering trust and smoother, more cooperative interactions.

### 2. Model Predictive Path Integral (MPPI)
MPPI is a powerful and highly-parallelizable control algorithm. It operates by:
1.  Sampling thousands of random potential control sequences (rollouts) over a short time horizon.
2.  Simulating the future trajectory for each rollout using a dynamics model.
3.  Evaluating each trajectory against a cost function.
4.  Calculating a weighted average of the best control sequences to produce the optimal command for the robot.

### 3. The Integrated Cost Function
The innovation of this package lies in the cost function within `sm_mppi.py`, which guides the MPPI controller. It evaluates trajectories based on a weighted sum of four key factors:

* **Goal Cost**: Drives the robot toward its final destination.
* **Social Momentum Cost**: This is the key to socially-aware behavior. It heavily penalizes any control actions that would reverse the established passing side (i.e., invert the sign of the angular momentum) and rewards actions that reinforce it.
* **Dynamic Obstacle Cost**: Penalizes proximity to human agents to ensure safety.
* **Static Obstacle Cost**: Prevents collisions with walls and other static obstacles in the environment.

## System Architecture

The package consists of several ROS 2 nodes that work together to create a complete simulation and planning environment.

* **`mppi_planner_node`**: The main planner node. It subscribes to a goal, uses TF2 to get the current state of the robot and humans, runs the SM-MPPI controller to calculate the best velocity, and publishes it as a `Twist` message.
* **`fake_human_publisher`**: A simulator node that spawns virtual humans. It publishes their positions to the `/tf` tree and provides visualization markers for RViz. It can operate in several scenarios (`random`, `head_on`, `teleop`).
* **`goal_publisher_node`**: A simple utility node to publish a target goal pose for the planner.

**Key Topics & Transforms:**
* **`/tf`**: All pose information for the robot (`base_link`) and humans (`human_0`, `human_1`, etc.) relative to the `odom` frame is handled via TF transforms.
* **`/goal_pose`** (`geometry_msgs/PoseStamped`): The input goal for the planner.
* **`/mobile_base_controller/cmd_vel_unstamped`** (`geometry_msgs/Twist`): The final velocity command published for the robot controller.
* **`/human_markers`** (`visualization_msgs/MarkerArray`): Markers for visualizing simulated humans in RViz.
* **`/sm_mppi_planner/vis/rollouts`** (`visualization_msgs/MarkerArray`): Visualizes the thousands of trajectories being evaluated by MPPI in real-time.

## Dependencies

**System:**
* ROS 2 (Developed on Humble)
* Python 3.10+

**Python Packages:**
* `torch` (and `pytorch-mppi`)
* `numpy`
* `shapely`
* `rclpy`

**ROS Packages:**
* `geometry_msgs`
* `visualization_msgs`
* `nav_msgs`
* `tf2_ros`
* `my_social_nav_interfaces` (custom message package included in this workspace)

## Installation & Building

1.  **Create and navigate to your ROS 2 workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2.  **Clone the Repository:**
    (Assuming this repository is cloned into the `src` folder)

3.  **Install Python Dependencies:**
    ```bash
    cd src/social_momentum_MPPI
    pip install -r requirements.txt 
    # (Note: A requirements.txt would need to be created with torch, pytorch-mppi, shapely, numpy)
    ```

4.  **Build the Workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

5.  **Source the Workspace:**
    ```bash
    source install/setup.bash
    ```

## Usage: Running the Simulation

The package includes several launch files to run different scenarios. The main simulation can be started with:

```bash
ros2 launch sm_mppi_planner sm_mppi_simulation.launch.py
```

This will typically launch:
* **RViz**: The ROS visualization tool.
* **`mppi_planner_node`**: The planner itself.
* **`fake_human_publisher`**: The human simulator.

Other launch files are provided for specific demonstrations:
* `headon_demo.launch.py`: A scenario where a human walks directly towards the robot.
* `teleop_launch.launch.py`: A scenario where a human can be controlled via a `Twist` topic, for example with `teleop_twist_keyboard`.

## Package Structure

```
social_momentum_MPPI/
├── my_social_nav_interfaces/     # Custom ROS2 Messages
│   ├── msg/
│   │   ├── HumanArray.msg
│   │   └── HumanPoseVel.msg
│   └── ...
└── src/
    └── sm_mppi_planner/
        ├── launch/                 # Launch files for different scenarios
        ├── package.xml             # ROS2 package manifest
        ├── setup.py                # Python package setup script
        └── sm_mppi_planner/
            ├── config.py           # Core parameters (velocities, horizon, etc.)
            ├── mppi_planner_node.py# Main executable ROS2 node
            ├── sm_mppi.py          # The SMMPPIController class with cost function
            ├── fake_human_publisher.py # Simulates human agents
            ├── goal_publisher_node.py # Publishes a sample goal
            ├── tf2_wrapper.py      # Helper class for TF2 transformations
            ├── utils.py            # Utility functions (dynamics model, etc.)
            └── vis_utils.py        # RViz visualization helpers
```

## Configuration

Key planner parameters can be adjusted in `sm_mppi_planner/config.py`:
* `VMAX`: Maximum linear velocity of the robot.
* `HORIZON_LENGTH`: Number of timesteps MPPI predicts into the future.
* `NUM_SAMPLES`: Number of trajectories sampled per cycle. High values improve performance but increase computational load.
* `ACTIVE_AGENTS`: The number of simulated humans to expect.

## Academic Context & Citation

This work is grounded in academic research. If you use this code or its concepts in your research, please cite the relevant publications.

**Thesis Context:**
> Landuyt, N. W. (2024). *Value sensitive design in Engineering* [Unpublished master's thesis]. Delft University of Technology. 

**Core Scientific Paper:**
> Mavrogiannis, C., Alves-Oliveira, P., Thomason, W., & Knepper, R. A. (2022). Social Momentum: Design and Evaluation of a Framework for Socially Competent Robot Navigation. *ACM Transactions on Human-Robot Interaction, 11*(2), 1–37. [https://doi.org/10.1145/3495244](https://doi.org/10.1145/3495244) 
