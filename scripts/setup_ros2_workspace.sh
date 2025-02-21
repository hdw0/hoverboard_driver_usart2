#!/bin/bash
source /opt/ros/humble/setup.bash
sudo chmod +x ./scripts/setup_ros2_workspace.sh
chmod +x ./scripts/setup_ros2_workspace.sh
colcon build
source install/setup.bash
