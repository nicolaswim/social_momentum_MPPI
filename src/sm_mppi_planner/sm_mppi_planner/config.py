import numpy as np
# import torch # Not strictly needed in config.py itself if not used for tensor creation here
from datetime import datetime

NOW = datetime.now()

# --- Core MPPI and Robot Parameters ---
VMAX = 0.3  # Maximum linear velocity the robot can command (m/s). TIAGo's actual max might be higher, but this is a safe start.
DT = 0.2    # Time step for each action in the MPPI prediction horizon (seconds).
HORIZON_LENGTH = 15 # Number of time steps MPPI looks into the future (DT * HORIZON_LENGTH = total prediction time).
NUM_SAMPLES = 250 # Number of trajectory samples MPPI generates.

# --- Goal Configuration ---
# MODIFIED: Single, clear goal further away to ensure perceivable movement.
# Robot usually spawns near (0,0). Let's send it to (X=2.0, Y=1.0).
GOALS = np.array([[2.0, 1.0]])

NUM_CYCLES = 1          # How many times to cycle through the GOALS list.
REPEAT_GOALS = False    # If True, loops back to the first goal after the last one in a cycle. Set to False for a single goal test.
TERMINATION_TOLERANCE = 0.2 # MODIFIED: Robot is considered to have reached the goal if within this distance (meters). Smaller for more precision.

# --- Social/Agent Configuration ---
# MODIFIED: Start with 0 active agents to remove social complexity for initial movement test.
ACTIVE_AGENTS = 0
# AGENT_GOALS = np.array([[0.0,-2.35], [0.0, 2.75]]) # Not used if ACTIVE_AGENTS = 0
HUMAN_FRAME = "human" # Base name for TF frames of humans (e.g., human_1, human_2)

# --- Obstacle Configuration ---
# MODIFIED: Start with NO static obstacles to ensure they are not blocking the robot.
STATIC_OBSTACLES = [] # Empty list means no static obstacles are considered by the planner.
# Original obstacles:
# [
#     [(-0.7, -2.0), (-2.0, -2.0), (-2.0, 2.0), (-0.7, 2.0)],
#     [(0.7, -2.0), (2.0, -2.0), (2.0, 2.0), (0.7, 2.0)]
# ]

# --- Other Parameters (Review if these are actively used by your SMMPPIController) ---
# HZ = 0.05 # This was in your original config.
            # Your mppi_planner_node.py uses a timer period of 0.05s (20Hz).
            # HZ here seems to be used for calculating agent velocities from TF.
            # If your planner loop is 20Hz, then HZ for velocity calc should match that period (0.05)
            # or you should use actual time deltas. Let's keep it for now.
HZ = 0.05 # Assuming this is the dt for velocity estimation of dynamic agents from TF.

# RADIUS = 0.2 # This might be the robot's radius for some calculation.
             # TIAGo's actual radius is closer to 0.275m.
             # The collision_avoidance_cost in your sm_mppi.py uses shapely polygons,
             # so this RADIUS might not be directly used for static obstacle collision there.
             # It could be used for dynamic agent collision or other parts.
RADIUS = 0.28 # MODIFIED: More realistic approximate radius for TIAGo if used for robot's own body.

# These seem less critical for initial movement but are kept from your original config.
# MAX_AGENT_NUM = 14 # Max number of agents the system might track (if you had more complex agent management)
# ANCA_CONTROL_POINTS = 5 # Unclear where this is used from the provided code snippets.
# USE_TERMINAL_COST = True # MPPI uses terminal_state_cost, so this is implicitly true.
ANGULAR_THRESHOLD = 0.3 # Radians, likely for determining if robot is "facing" the goal.

# NEED_ODOM = False # Your node *does* use odometry via TF for current_state.
# NEED_LASER = False # Not used in the provided MPPI code.
