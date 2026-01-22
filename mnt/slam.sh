#!/bin/bash
apt-get update
apt install ros-jazzy-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./my_slam_params.yaml
#ros2 launch slam_toolbox online_async_launch.py
