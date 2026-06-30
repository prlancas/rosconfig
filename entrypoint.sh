#!/bin/bash
# Source ROS 2 (and the slam_toolbox install) then exec the given command.
set -e
source /opt/ros/jazzy/setup.bash
exec "$@"
