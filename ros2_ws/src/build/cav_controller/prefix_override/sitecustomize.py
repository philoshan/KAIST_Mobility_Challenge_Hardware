import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cav-06/KAIST_Mobility_Challenge_Hardware/ros2_ws/src/install/cav_controller'
