#!/usr/bin/env bash
# Record the session to ~/runs/<time>_<label> (MCAP). Ctrl+C to stop.
#   ./record.sh [label]
source /opt/ros/jazzy/setup.bash
OUT=$HOME/runs/$(date +%Y%m%dT%H%M%S)_${1:-run}
mkdir -p "$HOME/runs"
exec ros2 bag record -s mcap -o "$OUT" \
  /joint_states /imu_sensor_broadcaster/imu /cmd_vel /joy /rosout \
  /estop_controller/stop_status \
  /neural_controller_student/policy_output \
  /neural_controller_student/policy_inference_latency_seconds
