import sys
if sys.prefix == '/home/wim/Documents/social_momentum_venv/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wim/Documents/social_momentum_venv/social_momentum_MPPI/install/sm_mppi_planner'
