import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hehe/Documents/GitHub/swarm_robots/simulation_ws/src/install/nav2_project'
