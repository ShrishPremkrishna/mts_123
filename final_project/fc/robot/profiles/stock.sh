# Profile "stock": the class platform as installed on the Pi (pupperv3-monorepo).
#   ros2 launch neural_controller launch.py teleop:=False      (the launch that walked 25 min on 2026-11-19)
#   + teleop_twist_joy remapped to /teleop_cmd_vel             (gamepad -> mux priority 0)
#   + cmd_vel_mux_node (500 ms timeout, deadband 0.05)         (mux -> /cmd_vel -> walker)
# The launch.py INSTALLED on the Pi starts no cmd_vel_mux and honours teleop:=False (checked on mts
# 2026-11-23: install/.../launch.py == src/.../launch.py; a laptop monorepo copy that starts a mux is a
# different revision). READY also requires the live graph to show exactly one /cmd_vel publisher.
# Stock driver (no 100 ms stale-IMU kill), stock estop_controller (X = neural_controller_default,
# R3 = e-stop, Options = resume), default_policy.json sha ecf3a4ab (same policy as tej_u6).
# Sourced by fcctl.sh, which provides fc_spawn / fc_alive / fc_signal_group / fc_log.

PROFILE_DESC="pupperv3-monorepo neural_controller launch.py + teleop->mux"
PROFILE_CMD_TOPIC=/llm_cmd_vel
PROFILE_IMU_TOPIC=/imu_sensor_broadcaster/imu
PROFILE_WALKER=neural_controller_default
# fcctl ready stands it in software: the stock estop_controller X sends exactly this SwitchController request
# (activate the walker, deactivate the other leg controllers, BEST_EFFORT) straight to controller_manager.
PROFILE_STAND_HINT="press X on the gamepad"
PROFILE_LEG_CONTROLLERS="neural_controller neural_controller_default neural_controller_default_alt neural_controller_three_legged neural_controller_student neural_controller_liedown"
# Gentle lie-down (fcctl down): an inactive second NeuralController instance that lowers the body to a lying
# pose before the legs are switched off (stack_overlay.py --liedown, sim/liedown_sim.py). Loaded by default;
# FC_LIEDOWN=0 starts without it (then down drops like R3, and with no other option the launch is unchanged).
PROFILE_LIEDOWN=neural_controller_liedown
# Lean launch (fcctl --lean on|off, --camera on|off): by default the overlay launch does not start foxglove_bridge
# or camera_ros (1400x1050 RGB888): FC v1 uses neither, and the Pi runs hot (BNO085 gyro fault at ~75 C on
# 2026-11-24). --camera on keeps the onboard camera; --lean off = the stock node list.
PROFILE_LEAN=on
PROFILE_CAMERA=off
MONO=/home/pi/pupperv3-monorepo
STOCK_CONFIG=$MONO/ros2_ws/install/neural_controller/share/neural_controller/launch/config.yaml
# Heading hold (the walker's yaw hold, see imu/heading_hold.py): on, unless an alias (stock_hh0.sh) set
# FC_HEADING_HOLD=0 before sourcing this file, or fcctl got --heading-hold off. Stack options (heading hold
# off, --speed-cap, the lie-down controller, lean) = profile_start first builds and verifies a fresh overlay in
# RUN_DIR/overlay (fcctl.sh fc_stack_overlay -> stack_overlay.py) and refuses to start if it cannot.
case "${FC_HEADING_HOLD:-1}" in
  1) PROFILE_HEADING_HOLD=on ;;
  0) PROFILE_HEADING_HOLD=off; PROFILE_DESC+=" + heading hold OFF (kp=0 overlay)" ;;
  *) PROFILE_HEADING_HOLD="invalid:$FC_HEADING_HOLD" ;;   # fcctl load_profile refuses it
esac

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
  local launch=(neural_controller launch.py) liedown=on
  [ "${FC_LIEDOWN:-1}" = 0 ] && liedown=off
  if [ "$liedown" = on ] || fc_stack_options_set; then
    # launch_fc.py = the installed launch.py (checked byte-identical to src) reading the overlay config.yaml
    # (+ the lie-down spawner); teleop keeps $STOCK_CONFIG (unchanged).
    fc_stack_overlay stock "$run" "$liedown" || return 1
    launch=("$OVERLAY_LAUNCH")
  fi
  fc_spawn launch "$run/robot.log" ros2 launch "${launch[@]}" teleop:=False
  fc_spawn teleop "$run/teleop.log" ros2 run teleop_twist_joy teleop_node --ros-args \
    --params-file "$STOCK_CONFIG" -r cmd_vel:=/teleop_cmd_vel
  fc_spawn mux "$run/mux.log" ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args \
    -p timeout_ms:=500 -p deadband:=0.05
  local deadline=$((SECONDS + 100))
  while [ $SECONDS -lt $deadline ]; do
    fc_alive launch || { fc_log "launch exited before ready"; return 1; }
    fc_alive teleop || { fc_log "teleop exited before ready; see teleop.log"; return 1; }
    fc_alive mux || { fc_log "cmd_vel_mux exited before ready; see mux.log"; return 1; }
    if grep -q "process has died" "$run/robot.log"; then fc_log "a launch process died"; return 1; fi
    if grep -q "Finished homing" "$run/robot.log" &&
       grep -q "Configured and activated.*imu_sensor_broadcaster" "$run/robot.log" &&
       grep -q "Loaded.*neural_controller_default" "$run/robot.log" &&
       { [ "$liedown" = off ] || grep -q "Loaded.*neural_controller_liedown" "$run/robot.log"; }; then
      fc_topology_wait $((deadline > SECONDS ? deadline - SECONDS : 1)) && return 0
      fc_log "velocity topology wrong (see above): refusing READY"; return 1
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
