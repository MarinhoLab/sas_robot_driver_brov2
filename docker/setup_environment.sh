#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws

rm -rf build install log
colcon build
source install/setup.bash

echo "source /root/ros2_ws/install/setup.bash"\
>> /etc/bash_env
