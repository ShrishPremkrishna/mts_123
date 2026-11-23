# Profile "stock": the class platform as installed on the Pi (pupperv3-monorepo).
#   ros2 launch neural_controller launch.py teleop:=False      (the launch that walked 25 min on 2026-11-19)
#   + teleop_twist_joy remapped to /teleop_cmd_vel             (gamepad -> mux priority 0)
#   + cmd_vel_mux_node (500 ms timeout, deadband 0.05)         (mux -> /cmd_vel -> walker)
# Stock driver (no 100 ms stale-IMU kill), stock estop_controller (X = neural_controller_default,
# R3 = e-stop, Options = resume), default_policy.json sha ecf3a4ab (same policy as tej_u6).
# Sourced by fcctl.sh, which provides fc_spawn / fc_alive / fc_signal_group / fc_log.

PROFILE_DESC="pupperv3-monorepo neural_controller launch.py + teleop->mux"
PROFILE_CMD_TOPIC=/llm_cmd_vel
PROFILE_IMU_TOPIC=/imu_sensor_broadcaster/imu
PROFILE_WALKER=neural_controller_default
PROFILE_LEG_CONTROLLERS="neural_controller neural_controller_default neural_controller_default_alt neural_controller_three_legged neural_controller_student"
MONO=/home/pi/pupperv3-monorepo
STOCK_CONFIG=$MONO/ros2_ws/install/neural_controller/share/neural_controller/launch/config.yaml

profile_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "$MONO/ros2_ws/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
  unset ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION
}

# profile_start RUN_DIR : runs inside the detached starter; returns 0 when homed + controllers loaded.
profile_start() {
  local run=$1
  profile_env
  [ -f "$STOCK_CONFIG" ] || { fc_log "missing $STOCK_CONFIG"; return 1; }
  fc_spawn launch "$run/robot.log" ros2 launch neural_controller launch.py teleop:=False
  fc_spawn teleop "$run/teleop.log" ros2 run teleop_twist_joy teleop_node --ros-args \
    --params-file "$STOCK_CONFIG" -r cmd_vel:=/teleop_cmd_vel
  fc_spawn mux "$run/mux.log" ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args \
    -p timeout_ms:=500 -p deadband:=0.05
  local deadline=$((SECONDS + 100))
  while [ $SECONDS -lt $deadline ]; do
    fc_alive launch || { fc_log "launch exited before ready"; return 1; }
    if grep -q "process has died" "$run/robot.log"; then fc_log "a launch process died"; return 1; fi
    if grep -q "Finished homing" "$run/robot.log" &&
       grep -q "Configured and activated.*imu_sensor_broadcaster" "$run/robot.log" &&
       grep -q "Loaded.*neural_controller_default" "$run/robot.log"; then
      return 0
    fi
    sleep 0.5
  done
  fc_log "not ready within 100 s"
  return 1
}

profile_alive() { fc_alive launch; }

# profile_stop RUN_DIR : fcctl has already zeroed velocity and deactivated leg controllers.
profile_stop() {
  local run=$1 rc=0
  fc_signal_group teleop INT 5 || rc=1
  fc_signal_group mux INT 5 || rc=1
  # Same as Ctrl+C on the launch terminal (the 2026-11-19 shutdown path).
  fc_signal_group launch INT 25 || rc=1
  return $rc
}
