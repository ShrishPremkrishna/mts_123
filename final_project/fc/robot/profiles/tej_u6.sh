# Profile "tej_u6": Tej's ball-kick release launched exactly the way his kit does it
# (kickseen-kit-20261120/kickseen6-{prepare,session}.sh: persistent_session.py prepare + start-native
# -> lab6_session.launch.py, ROS_DOMAIN_ID=168, driver 1dd92b65, native 37938659, gateway estop_controller,
# cmd_vel_mux with /teleop_cmd_vel > /llm_cmd_vel > /person_following_cmd_vel, 500 ms).
# Only the run directory differs (fresh one under ~/fc-v1/runs/). We do NOT start his HTTP tool API
# (start-api); our drive_bridge publishes /llm_cmd_vel instead, which nothing else publishes without it.
# Additive: a teleop_twist_joy node remapped to /teleop_cmd_vel so the gamepad sticks override the bridge
# (his launch has no teleop node; disable with FC_TELEOP=0).
# His launch already starts the one cmd_vel_mux (lab6_session.launch.py, mux is a "required" node:
# its exit tears the session down). The Lab 5 follower publishes /person_following_cmd_vel (lowest).
# READY requires the live graph to show exactly one /cmd_vel publisher (cmd_vel_mux).
# Physical: start-native homes the legs. Gateway starts LATCHED: press Options, then X
# (neural_controller_default) on the DualSense to stand; R3 = e-stop.
# Sourced by fcctl.sh, which provides fc_spawn / fc_alive / fc_signal_group / fc_log.

PROFILE_DESC="Tej software-ballkick-20261120-u6 via kickseen6 kit (no API)"
PROFILE_CMD_TOPIC=/llm_cmd_vel
PROFILE_IMU_TOPIC=/imu_sensor_broadcaster/imu
PROFILE_WALKER=neural_controller_default
# Tej's Stop gateway owns /controller_manager/switch_controller, starts LATCHED and refuses any software
# activation until a physical Options press (joy_utils stop_arbiter.hpp submit(); no software release API
# in u6). Options alone activates controller_names[0] = neural_controller_default. fcctl ready then waits.
PROFILE_STAND_HINT="press Options on the gamepad (Stop gateway latched; Options releases it and stands the walker)"
PROFILE_LEG_CONTROLLERS="neural_controller neural_controller_default neural_controller_default_alt neural_controller_three_legged neural_controller_student gemini_walk trick_leg_kick trick_sit_stay_shake trick_spider_walk walk_spider_walk trick_swim"
TEJ_BASE=/home/pi/pupper-foundation-session-20261029
TEJ_KIT=$TEJ_BASE/kickseen-kit-20261120
TEJ_RELEASE=$TEJ_BASE/software-ballkick-20261120-u6
TEJ_YAML=$TEJ_RELEASE/foundation/hardware/lab6-session.yaml
# sha256 of the kit scripts we derive from (observed on mts 2026-11-23).
TEJ_PREPARE_SHA=6dea87f4636c15c825e9ebd3f6aae45d03b530dc133fb2d71995c27f09813417
TEJ_SESSION_SHA=f5913e5b48285b71b981c0265e9efae599ba0312deffb162f6f47fbbfa92f099
# Heading hold: on, unless an alias (tej_u6_hh0.sh) set FC_HEADING_HOLD=0 before sourcing this file, or fcctl got
# --heading-hold off. Stack options (heading hold off, --speed-cap) = profile_start first builds and verifies a
# fresh overlay of Tej's launch dir in RUN_DIR/overlay (fcctl.sh fc_stack_overlay -> stack_overlay.py u6).
# Limits (pinned kit): lab6-session.yaml stays byte-identical (sha-pinned in his launch); the cap is set on
# neural_controller_default + gemini_walk in the launch file's in-memory baseline, and his trick controllers
# derived from that baseline inherit it. No gentle lie-down controller: down drops like R3.
case "${FC_HEADING_HOLD:-1}" in
  1) PROFILE_HEADING_HOLD=on ;;
  0) PROFILE_HEADING_HOLD=off; PROFILE_DESC+=" + heading hold OFF (kp=0 overlay)" ;;
  *) PROFILE_HEADING_HOLD="invalid:$FC_HEADING_HOLD" ;;   # fcctl load_profile refuses it
esac

profile_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
  export ROS_DOMAIN_ID=168 ROS_LOCALHOST_ONLY=1
  unset ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION
}

# Copy a kit script with only RUN_DIR changed. Refuses if the kit script changed since it was pinned.
# With HW (heading hold off, prepare script only) the --launch/--config/--urdf arguments also point at the
# overlay's foundation/hardware/ (siblings, as persistent_session.py requires); nothing else changes.
_tej_derive() {
  local src=$1 sha=$2 dst=$3 session=$4 hw=${5:-} want=1
  [ "$(sha256sum "$src" | cut -d' ' -f1)" = "$sha" ] || { fc_log "kit script changed: $src"; return 1; }
  if [ -n "$hw" ]; then
    want=4
    sed -E -e "s|^RUN_DIR=.*|RUN_DIR=\"$session\"|" \
      -e 's#^(  --(launch|config|urdf) )"\$APP_RELEASE/foundation/hardware/#\1"'"$hw"'/#' "$src" > "$dst"
    [ "$(grep -cF -- " \"$hw/" "$dst")" = 3 ] || { fc_log "overlay --launch/--config/--urdf rewrite failed: $dst"; return 1; }
  else
    sed "s|^RUN_DIR=.*|RUN_DIR=\"$session\"|" "$src" > "$dst"
  fi
  [ "$(grep -c "^RUN_DIR=\"$session\"$" "$dst")" = 1 ] || { fc_log "RUN_DIR rewrite failed: $dst"; return 1; }
  [ "$(diff "$src" "$dst" | grep -c '^>')" = "$want" ] || { fc_log "derived $dst does not change exactly $want line(s); refusing"; return 1; }
  if grep -v '^#' "$dst" | grep -q 'supervised-console-runs'; then fc_log "stale run path in $dst"; return 1; fi
}

profile_start() {
  local run=$1 session=$1/session hw=""
  if fc_stack_options_set; then
    # Overlay of Tej's pinned launch dir: lab6_session.launch.py patched (kp=0 policy sha-pinned inside it,
    # speed caps), byte-identical lab6-session.yaml / URDF. The kit scripts are still checked against their pins.
    fc_stack_overlay u6 "$run" off || return 1
    hw=$OVERLAY_HW
  fi
  _tej_derive "$TEJ_KIT/kickseen6-prepare.sh" "$TEJ_PREPARE_SHA" "$run/prepare.sh" "$session" "$hw" || return 1
  _tej_derive "$TEJ_KIT/kickseen6-session.sh" "$TEJ_SESSION_SHA" "$run/session.sh" "$session" || return 1
  fc_log "prepare (no motion)"
  bash "$run/prepare.sh" > "$run/prepare.log" 2>&1 || { fc_log "prepare failed; see prepare.log"; return 1; }
  fc_log "start-native (HOMES THE LEGS; 3 s still IMU check first)"
  bash "$run/session.sh" start-native > "$run/start-native.log" 2>&1 ||
    { fc_log "start-native failed; see start-native.log and session/robot.log"; return 1; }
  if [ "${FC_TELEOP:-1}" = 1 ]; then
    profile_env
    fc_spawn teleop "$run/teleop.log" ros2 run teleop_twist_joy teleop_node --ros-args \
      --params-file "$TEJ_YAML" -r cmd_vel:=/teleop_cmd_vel
  fi
  profile_env
  fc_topology_wait 30 || { fc_log "velocity topology wrong (see above): refusing READY"; return 1; }
  return 0
}

profile_alive() {
  local run=$1
  [ -f "$run/session/owned_session.json" ] && pgrep -f "ros2_control_node" > /dev/null
}

# fcctl has already zeroed velocity and deactivated leg controllers (through the gateway).
profile_stop() {
  local run=$1 rc=0
  fc_signal_group teleop INT 5 || rc=1
  if [ -f "$run/session.sh" ] && [ -f "$run/session/owned_session.json" ]; then
    # Tej's own manager-first shutdown of the processes his session owns (camera, recorder, launch).
    bash "$run/session.sh" shutdown >> "$run/shutdown.log" 2>&1 || { fc_log "session shutdown failed; see shutdown.log"; rc=1; }
  fi
  return $rc
}
