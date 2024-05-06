#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/mira_ws/devel/setup.bash"

# Run additional commands or start your ROS nodes here
# For example:
# roscore &
# rosrun my_package my_node

# If you just want to start a bash shell
exec "$@"