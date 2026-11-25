# Profile "stock_hh0": profile "stock" with the walker's heading hold off (alias of --heading-hold off; lane D: the same default policy
# with heading_hold.kp = 0, fc/notes/gyro.md). All logic is stock.sh; its profile_start builds and
# verifies a fresh overlay in RUN_DIR/overlay before launching and refuses to start if it cannot.
# Sourced by fcctl.sh (load_profile), which provides fc_spawn / fc_alive / fc_stack_overlay / fc_log.
FC_HEADING_HOLD=0
# shellcheck source=stock.sh
source "$(dirname "${BASH_SOURCE[0]}")/stock.sh"
