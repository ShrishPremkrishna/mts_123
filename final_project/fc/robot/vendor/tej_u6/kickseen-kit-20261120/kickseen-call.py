#!/usr/bin/env python3
"""Call one API tool and print its result. A tool that starts a job (kick_seen, stand_up, move) is
followed until the job ends; look_ball answers at once.

    kickseen-call.py look_ball
    kickseen-call.py kick_seen '{"side": "right"}'

A Ctrl-C or an error sends Stop to the robot. A job that fails does not need a Stop: it has ended.
"""
import json
import sys
import time

sys.path.insert(0, "/home/pi/pupper-foundation-session-20261029/software-ballkick-20261120-u6/gemini_pupper/robot")
from ball_seek import HttpClient  # noqa: E402

tool = sys.argv[1]
args = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
client = HttpClient()
stamp = lambda text: print(time.strftime("%H:%M:%S"), text, flush=True)  # noqa: E731
try:
    res = client.call(tool, args)
    job = ((res.get("details") or {}).get("submission") or {}).get("job_id")
    if not res.get("success") or not job:
        stamp(("ANSWER: " if res.get("success") else "REFUSED: ") + json.dumps(res, indent=1)[:6000])
        sys.exit(0 if res.get("success") else 2)
    stamp(f"job {job} started")
    end = time.monotonic() + 90.0
    while time.monotonic() < end:
        time.sleep(0.2)
        st = client.status()
        if st.get("current_action") is None and not st.get("queued"):
            last = st.get("last_outcome") or {}
            if last.get("job_id") != job:
                continue
            stamp(("DONE: " if last.get("success") else "FAILED: ") + json.dumps(last, indent=1)[:6000])
            sys.exit(0 if last.get("success") else 1)
    client.stop()
    stamp("TIMEOUT after 90 s: Stop sent")
    sys.exit(1)
except KeyboardInterrupt:
    client.stop()
    stamp("STOPPED (Ctrl-C): Stop sent")
    sys.exit(130)
except Exception as error:
    client.stop()
    stamp(f"ERROR: {type(error).__name__} {error} (Stop sent)")
    sys.exit(1)
