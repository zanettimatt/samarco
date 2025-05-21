#!/bin/bash
# This script is used to run the automation robot teleop node in a ROS 2 environment.
source automation_ws/install/setup.bash
ros2 run automation_robot_teleop_pkg keyboard_teleop
