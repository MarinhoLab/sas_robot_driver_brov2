#!/bin/bash
# Launch the Blue simulation (BlueROV2) headless. Run inside the blue-sim container, e.g.:
#   docker cp .agents/launch_blue.sh blue-sim:/tmp/launch_blue.sh
#   docker exec -d blue-sim bash -lc 'bash /tmp/launch_blue.sh'
# Usage: launch_blue.sh [bluerov2|bluerov2_heavy]   (default: bluerov2)
model="${1:-bluerov2}"
export HOME=/home/ubuntu
export PATH=$HOME/ardupilot/build/sitl/bin:$PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws_blue/install/setup.bash
env -u LIBGL_ALWAYS_SOFTWARE GZ_IP=127.0.0.1 ros2 launch blue_bringup "${model}.launch.yaml" use_sim:=true > /tmp/blue.log 2>&1

