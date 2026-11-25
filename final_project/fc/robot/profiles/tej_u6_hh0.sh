# Profile "tej_u6_hh0": profile "tej_u6" with the walker's heading hold off (alias of --heading-hold off; lane D: the same pinned policy
# with heading_hold.kp = 0, fc/notes/gyro.md). All logic is tej_u6.sh: the kit scripts are still
# checked against their sha pins; only prepare's --launch/--config/--urdf point at a fresh, verified
# overlay in RUN_DIR/overlay. Refuses to start if the overlay cannot be built.
# Sourced by fcctl.sh (load_profile), which provides fc_spawn / fc_alive / fc_stack_overlay / fc_log.
FC_HEADING_HOLD=0
# shellcheck source=tej_u6.sh
source "$(dirname "${BASH_SOURCE[0]}")/tej_u6.sh"
