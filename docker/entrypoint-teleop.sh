#!/bin/bash

# Configuração do ambiente ROS
source /opt/ros/humble/setup.bash

cd automation_ws

colcon build

# Se o workspace já foi compilado, carregue-o
if [ -f "./install/setup.bash" ]; then
  source ./install/setup.bash
fi

ros2 run teleop_twist_keyboard teleop_twist_keyboard