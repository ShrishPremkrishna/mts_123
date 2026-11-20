#!/usr/bin/env python3
"""Print the field state as it arrives. This is all a consumer needs to do."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.config import load_config  # noqa: E402
from fieldcam.net import StateSubscriber  # noqa: E402

sub = StateSubscriber(load_config()["net"]["port"])
while True:
    st = sub.latest()
    if st:
        b = st["ball"]
        line = f"seq {st['seq']:6d} rx_age {st['rx_age_s'] * 1000:4.0f}ms  "
        line += f"ball ({b['x']:+.2f},{b['y']:+.2f}) v({b['vx']:+.2f},{b['vy']:+.2f})  " if b["visible"] else "ball LOST  "
        for name, r in st["robots"].items():
            line += f"{name} ({r['x']:+.2f},{r['y']:+.2f},{r['yaw']:+.2f})  " if r["visible"] else f"{name} LOST  "
        rel = st.get("relative", {}).get("striker", {}).get("ball")
        if rel:
            line += f"ball from striker: {rel['x']:+.2f} ahead {rel['y']:+.2f} left, bearing {rel['bearing']:+.2f} rad  "
        s = st["shots"].get("striker", {})
        if "cmd" in s:
            line += f"cmd vx {s['cmd']['vx']:+.2f} vy {s['cmd']['vy']:+.2f} wz {s['cmd']['wz']:+.2f} ready {s['ready']}"
        print(line)
    time.sleep(0.1)
