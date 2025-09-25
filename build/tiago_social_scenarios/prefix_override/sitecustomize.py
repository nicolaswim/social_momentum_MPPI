import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wim/Documents/social_momentum_venv/social_momentum_MPPI/install/tiago_social_scenarios'
