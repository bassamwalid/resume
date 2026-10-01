import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yassin-fakhry/ros2_ws/install/Autonomous_Systems_Project_Team_18'
