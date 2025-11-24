import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gabrielmartins/Documents/github/ponderada_ros/install/cg_teleop'
