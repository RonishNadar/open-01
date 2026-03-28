import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/open-01/robot_ros2/ros2_ws/install/open01_bringup'
