"""Workshop entry point: live gait tuner wired to the real Pupper.

Bring up the ros2_control stack the same way you do for lab 4 (so that
/forward_command_controller exists), then run:

    python3 main.py

Open http://<pi-ip>:8080 in a browser. Sliders update the 3D view instantly;
tick "Send to robot" to drive the motors (ramps in smoothly from a stand pose).
"""

import time
from ros_bridge import spin_bridge_in_thread
from viser_app import GaitTunerApp


def main():
    bridge = spin_bridge_in_thread()
    app = GaitTunerApp(robot=bridge)
    print("Gait tuner running — open http://<pi-ip>:8080  ('Send to robot' off by default)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.disable()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
