#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/open01/.venv/bin/activate
source /home/open01/open-01/robot_ros2/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# Serial bridge + bringup
ros2 launch open01_bringup bringup.launch.py &

sleep 5

# Rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

sleep 2

# OLED
python3 /home/open01/open-01/robot_ros2/oled_status.py &

wait
