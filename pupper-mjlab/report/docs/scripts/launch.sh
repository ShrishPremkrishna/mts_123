#!/usr/bin/env bash
# Start the robot stack (homes the legs; all policy controllers start inactive).
#   ./launch.sh [launch_arg:=value ...]
MONO=${MONO:-$HOME/pupperv3-monorepo}
source /opt/ros/jazzy/setup.bash
source "$MONO/ros2_ws/install/setup.bash"
exec ros2 launch neural_controller launch.py "$@"
