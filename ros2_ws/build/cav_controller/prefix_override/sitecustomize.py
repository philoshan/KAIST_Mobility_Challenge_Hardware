import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hy/Mobility_Challenge/ros2_ws/install/cav_controller'
