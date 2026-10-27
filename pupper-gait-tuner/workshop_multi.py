"""Multi-group workshop launcher.

Runs ONE shared robot bridge and N independent gait tuners, each on its own port,
so several groups can tune their gait at the same time and deploy one-by-one.

Only one group can drive the motors at a time. Clicking "Send to robot" takes
exclusive control and bumps whoever had it (their checkbox unticks and they see
who's now driving). Everyone can keep tuning in visualize-only mode meanwhile.

Usage:
    python3 workshop_multi.py            # 3 groups on ports 8080, 8081, 8082
    python3 workshop_multi.py 4          # 4 groups on ports 8080..8083
    python3 workshop_multi.py 4 9000     # 4 groups starting at port 9000

Each group opens:  http://<robot-ip>:<their port>
"""

import sys
import time

from ros_bridge import spin_bridge_in_thread
from viser_app import GaitTunerApp, get_tailscale_ip, get_lan_ip


def main():
    n_groups = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    base_port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

    bridge = spin_bridge_in_thread()
    apps = []
    for i in range(n_groups):
        port = base_port + i
        name = f"Group {i + 1}"
        apps.append(GaitTunerApp(robot=bridge, port=port, name=name))

    ip = get_tailscale_ip() or get_lan_ip() or "<robot-ip>"
    line = "=" * 60
    print(f"\n{line}")
    print(f"  {n_groups} gait tuners running. Hand each group ONE URL:")
    for i in range(n_groups):
        print(f"    • Group {i + 1}:  http://{ip}:{base_port + i}")
    print(f"\n  Only one group drives the motors at a time —")
    print(f"  'Send to robot' takes control from whoever had it.")
    print(f"{line}\n", flush=True)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.stand()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
