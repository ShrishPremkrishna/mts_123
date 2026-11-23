"""Robot readiness: one phase per robot for the laptop's traffic light (pure, stdlib only).

fcctl.sh `status` feeds its raw facts to phase(); `ready` uses legs() on the rosq JSON.

Phases (in power-on order):
  off       no fcctl profile and no robot stack (or only state left over from an earlier boot)
  starting  the profile's starter is running (homing, controllers loading, topology check)
  homed     READY: homed, controllers loaded, mux ok, no leg controller active (lying / limp / on the stand)
  standing  a leg controller is active, bridge not running
  ready     a leg controller is active, bridge running, mux ok: the laptop can drive it
  fault     anything that needs a person: start failed, stack died, foreign stack, mux wrong,
            controller manager unreachable
Light: off -> grey, starting/homed/standing -> amber, ready -> green, fault -> red.

What it cannot see: the stock neural controller's own fall/e-stop flag. After a fall or R3 inside an
active controller the robot is limp while list_controllers still says "active" (R3 also deactivates,
so after R3 the phase drops to homed; a fall alone does not).
"""
import json
import sys

LIGHT = {"off": "grey", "starting": "amber", "homed": "amber", "standing": "amber",
         "ready": "green", "fault": "red"}


def legs(controllers, walker, leg_controllers):
    """controllers: {name: state} from list_controllers, or None if the manager did not answer."""
    if controllers is None:
        return {"reachable": False, "walker": None, "walker_active": False, "active_legs": []}
    active = [c for c in leg_controllers if controllers.get(c) == "active"]
    return {"reachable": True, "walker": controllers.get(walker), "walker_active": controllers.get(walker) == "active",
            "active_legs": active}


def phase(st):
    """st keys (missing = falsy): profile, profile_alive, starting, ready, start_failed, stale,
    foreign_stack, ros2_control_node, ros (rosq probe dict or None), bridge_alive, walker, leg_controllers."""
    def out(p, detail=""):
        return {"phase": p, "light": LIGHT[p], "detail": detail}

    if st.get("foreign_stack"):
        return out("fault", "a robot stack not started by fcctl is running")
    if not st.get("profile") or (st.get("stale") and not st.get("profile_alive") and not st.get("starting")):
        if st.get("ros2_control_node"):
            return out("fault", "ros2_control_node running without an fcctl profile")
        return out("off", "state from an earlier boot" if st.get("profile") else "")
    if st.get("starting"):
        return out("starting", "homing / loading controllers")
    if st.get("start_failed"):
        return out("fault", "start failed: see the run dir's starter.log, then fcctl.sh stop")
    if not st.get("profile_alive"):
        return out("fault", "profile %s is not running (stack exited): fcctl.sh stop" % st["profile"])
    if not st.get("ready"):
        return out("fault", "stack up but never reached READY: fcctl.sh stop")
    ros = st.get("ros") or {}
    ctl = legs(ros.get("controllers"), st.get("walker") or "", st.get("leg_controllers") or [])
    if not ctl["reachable"]:
        return out("fault", "controller manager not answering")
    mux = ros.get("mux") or {}
    if mux.get("ok") is False:
        return out("fault", "velocity topology: " + "; ".join(mux.get("problems") or ["mux not ok"]))
    if not ctl["active_legs"]:
        return out("homed", "lying/limp: Ready stands it")
    if st.get("bridge_alive"):
        return out("ready", "standing on %s, bridge up" % ",".join(ctl["active_legs"]))
    return out("standing", "standing on %s, bridge down" % ",".join(ctl["active_legs"]))


def main(argv):
    # readiness.py legs '<rosq json>' WALKER 'LEG1 LEG2 ...'  -> one JSON line
    if len(argv) == 4 and argv[0] == "legs":
        try:
            q = json.loads(argv[1])
        except ValueError:
            q = {}
        c = q.get("controllers") if isinstance(q, dict) else None
        res = legs(c if isinstance(c, dict) else None, argv[2], argv[3].split())
        res["mux_ok"] = ((q.get("mux") or {}).get("ok") if isinstance(q, dict) else None)
        res["switch_ok"] = q.get("ok") if isinstance(q, dict) else None
        print(json.dumps(res, separators=(",", ":")))
        return 0
    print("usage: readiness.py legs '<rosq json>' WALKER 'LEGS...'", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
