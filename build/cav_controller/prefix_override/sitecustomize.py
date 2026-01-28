import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chan/KAIST_Mobility_Challenge_Hardware/install/cav_controller'
