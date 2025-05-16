import numpy as np
from datetime import datetime

NOW = datetime.now()

# --- Core MPPI and Robot Parameters ---
# These will be used by mppi_planner_node.py to pass to SMMPPIController
# if not overridden by ROS parameters via a YAML file.
VMAX = 0.5  # Default max linear velocity
DT = 0.2    # Default time step for MPPI prediction horizon
HORIZON_LENGTH = 15 # Default number of time steps MPPI looks into the future
NUM_SAMPLES = 250 # Default number of trajectory samples

# --- Goal Configuration (for SMMPPIController's internal goal list if no external goal) ---
GOALS = np.array([[5.0, 0.0]]) # Default robot goal(s)
NUM_CYCLES = 1
REPEAT_GOALS = False
TERMINATION_TOLERANCE = 0.3
ANGULAR_THRESHOLD = 0.3 # For goal orientation checking

# --- Social/Agent Configuration (Defaults for mppi_planner_node.py) ---
ACTIVE_AGENTS = 2 # Default if not overridden by ROS parameters
AGENT_GOALS = np.array([0.0, -5.0]) # Default if not overridden and ACTIVE_AGENTS is 0
AGENT_GOALS_YAML_DEFAULT = "[]" # Default for the YAML string parameter in mppi_planner_node.py
HUMAN_FRAME = "human" # Default human frame prefix

# --- Obstacle Configuration (Default for mppi_planner_node.py) ---
STATIC_OBSTACLES = [] # Default static obstacles if not overridden by ROS parameters
STATIC_OBSTACLES_YAML_DEFAULT = "[]" # Default for the YAML string parameter

# --- Other Parameters ---
HZ = 0.05 # For TF-based velocity estimation in mppi_planner_node.py
RADIUS = 0.28 # Robot's approximate radius (used by SMMPPIController if imported)

# NOTE: GOAL_COST_WEIGHT, DYNAMIC_OBS_COST_WEIGHT, STATIC_COST_WEIGHT, HUMAN_RADIUS
# are NOT defined here because your provided sm_mppi.py uses hardcoded weights
# in its terminal_cost function (e.g., 2*goal_cost, 5*static_costs).
