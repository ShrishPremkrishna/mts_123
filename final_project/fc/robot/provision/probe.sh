#!/usr/bin/env bash
# probe.sh — READ-ONLY inspection of a Pupper Pi, run as `ssh <host> bash -s < probe.sh`.
# Prints one `key=value` line per fact for report.py. Writes nothing: no files, no git index refresh
# (--no-optional-locks), no .pyc (-B), no device opened (only `ls`/`test -e` on /dev), no ROS node.
# Every probe tolerates failure; a missing tool just yields an empty or "no" value.
set -u
export PYTHONDONTWRITEBYTECODE=1 LC_ALL=C
MONO=${FC_MONO:-$HOME/pupperv3-monorepo}
INST=$MONO/ros2_ws/install
FC=$HOME/fc-v1

kv() { printf '%s=%s\n' "$1" "$(printf '%s' "${2:-}" | tr '\n\t\r' '   ' | sed 's/ *$//')"; }
yn() { if "$@" >/dev/null 2>&1; then echo yes; else echo no; fi; }
sha12() { [ -f "$1" ] && sha256sum "$1" 2>/dev/null | cut -c1-12; }

kv probe_version 1
kv user "$(id -un)"
kv home "$HOME"
kv hostname "$(hostname 2>/dev/null)"
kv os "$(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
kv arch "$(uname -m)"
kv kernel "$(uname -r)"
kv model "$(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
kv uptime_s "$(cut -d. -f1 /proc/uptime 2>/dev/null)"

# ---- identity (a cloned card shares all of these with its source) ----
kv machine_id_hash "$( [ -r /etc/machine-id ] && sha256sum /etc/machine-id | cut -c1-12)"
kv ssh_hostkey_fp "$(ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub 2>/dev/null | awk '{print $2}')"
kv tailscale_installed "$(yn command -v tailscale)"
kv tailscaled_active "$(systemctl is-active tailscaled 2>/dev/null)"
kv tailscale_ip "$(timeout 5 tailscale ip -4 2>/dev/null | head -1)"
kv tailscale_name "$(timeout 5 tailscale status --json 2>/dev/null | python3 -B -c 'import json,sys; print(json.load(sys.stdin)["Self"]["HostName"])' 2>/dev/null)"
kv lan_ips "$(hostname -I 2>/dev/null)"
kv wifi_ssid "$(timeout 5 iwgetid -r 2>/dev/null || timeout 5 nmcli -t -f active,ssid dev wifi 2>/dev/null | awk -F: '$1=="yes"{print $2; exit}')"
kv appliance_mode "$(yn test -e /etc/fc-appliance-boot/appliance-mode)"
kv sudo_nopass "$(yn sudo -n true)"

# ---- ROS + class stack ----
kv ros_distros "$(ls /opt/ros 2>/dev/null | tr '\n' ' ')"
kv ros_jazzy "$(yn test -f /opt/ros/jazzy/setup.bash)"
kv mono "$(yn test -f "$INST/setup.bash")"
kv mono_rev "$(git --no-optional-locks -C "$MONO" rev-parse --short HEAD 2>/dev/null)"
kv mono_dirty_files "$(git --no-optional-locks -C "$MONO" diff --name-only HEAD 2>/dev/null | wc -l | tr -d ' ')"
for p in neural_controller control_board_hardware_interface cmd_vel_mux teleop_twist_joy joy controller_manager camera_ros; do
  if [ -d "$INST/$p" ]; then w=install; elif [ -d "/opt/ros/jazzy/share/$p" ]; then w=opt; else w=missing; fi
  kv "pkg_$p" "$w"
done
LAUNCH=$INST/neural_controller/share/neural_controller/launch/launch.py
CONF=$INST/neural_controller/share/neural_controller/launch/config.yaml
kv stock_config "$(yn test -f "$CONF")"
kv launch_py "$(yn test -f "$LAUNCH")"
# The stock profile starts its own cmd_vel_mux; a launch.py that also starts one = 2 muxes = never READY.
kv launch_starts_mux "$( [ -f "$LAUNCH" ] && { grep -q 'executable="cmd_vel_mux_node"' "$LAUNCH" && echo yes || echo no; })"
kv launch_teleop_arg "$( [ -f "$LAUNCH" ] && { grep -q 'LaunchConfiguration("teleop")' "$LAUNCH" && echo yes || echo no; })"
kv launch_sha "$(sha12 "$LAUNCH")"
pol=$(grep -m1 -E '^\s*model_path:' "$CONF" 2>/dev/null | sed -E 's/^\s*model_path:\s*//; s/["'\'']//g')
kv policy_path "$pol"
case $pol in /*) kv policy_sha "$(sha12 "$pol")" ;; *) kv policy_sha "" ;; esac
kv python "$(python3 -B -c 'import sys; print("%d.%d.%d" % sys.version_info[:3])' 2>/dev/null)"
kv rclpy "$( [ -f /opt/ros/jazzy/setup.bash ] && (set +u; . /opt/ros/jazzy/setup.bash >/dev/null 2>&1; python3 -B -c 'import rclpy, geometry_msgs.msg' >/dev/null 2>&1) && echo yes || echo no)"

# ---- hardware presence (listing only; nothing is opened) ----
kv spidev "$(ls /dev/spidev* 2>/dev/null | wc -l | tr -d ' ')"
kv i2c "$(ls /dev/i2c-* 2>/dev/null | wc -l | tr -d ' ')"
kv video "$(ls /dev/video* 2>/dev/null | wc -l | tr -d ' ')"
kv gamepad_paired "$(timeout 4 bluetoothctl devices Paired 2>/dev/null | grep -ciE 'wireless controller|dualsense|dualshock')"
kv disk_free_gb "$(df -Pk "$HOME" 2>/dev/null | awk 'NR==2{printf "%.1f", $4/1048576}')"
kv disk_total_gb "$(df -Pk / 2>/dev/null | awk 'NR==2{printf "%.1f", $2/1048576}')"
kv root_dev "$(findmnt -no SOURCE / 2>/dev/null)"
kv temp_c "$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)"

# ---- what is running (fcctl start refuses if a stack is already up) ----
kv running_control "$(pgrep -fc '[r]os2_control_node' 2>/dev/null)"
kv running_launch "$(pgrep -fc '[r]os2 launch' 2>/dev/null)"
kv running_session "$(pgrep -fc '[p]ersistent_session.py' 2>/dev/null)"
kv running_bridge "$(pgrep -fc '[d]rive_bridge' 2>/dev/null)"
kv robot_service "$(systemctl is-enabled robot.service 2>/dev/null)"

# ---- our files ----
kv fc_dir "$(yn test -d "$FC/robot")"
kv fc_robot_id "$(head -c 64 "$FC/robot/robot_id" 2>/dev/null)"
kv fc_link_key_mode "$(stat -c %a "$FC/robot/link.key" 2>/dev/null)"
kv fc_identity "$(head -c 400 "$FC/identity.json" 2>/dev/null)"
kv fc_active "$(head -c 200 "$FC/run/active" 2>/dev/null)"
kv fc_bridge_state "$(yn test -f "$FC/run/bridge_state.json")"
for f in drive_bridge.py fcctl.sh imu_health.py profiles/stock.sh; do kv "fc_sha_$f" "$(sha12 "$FC/robot/$f")"; done
kv probe_done yes
