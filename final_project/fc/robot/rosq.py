"""rosq.py — fcctl's ROS graph and controller queries (rclpy, no ros2 daemon). One compact JSON line.

  rosq.py probe [CMD_TOPIC]     velocity topology + controllers (fcctl status / ready / start)
  rosq.py controllers           controllers only
  rosq.py switch "<on...>" "<off...>"   SwitchController (BEST_EFFORT), then re-list controllers

Why the probe waits for the graph to settle (2026-11-23, mts): every call is a NEW DDS participant, and
discovery of a stack-sized graph (ros2_control_node alone has hundreds of endpoints) arrives in random order
over seconds. The old fixed 1.5 s spin missed cmd_vel_mux entirely in 3 of 12 probes on a stack-like graph
with the Pi loaded (domain 42 repro) and made `ready`'s verify step report "/cmd_vel publishers []" with
the mux running. Now: spin at least MIN_S, finish once the topology is complete and unchanged for QUIET_S,
give up at MAX_S and say why (mux process running but not discovered vs no mux process).

Host-wide DDS participants: under ROS_LOCALHOST_ONLY=1 rmw_fastrtps (Jazzy) announces only to the first 32
participant ids on 127.0.0.1 (udp maxInitialPeersRange = 32), and Fast DDS 2.14 allocates ids host-wide
across domains (one UDP port 7000+id each). Two participants with ids >= 32 never discover each other, so
the probe reports the count and warns near the limit.
"""
import json
import os
import sys
import time

MIN_S, QUIET_S, MAX_S = 2.0, 1.5, 8.0
MAX_INITIAL_PEERS = 32          # rmw_fastrtps_shared_cpp participant.cpp (jazzy), LOCALHOST range
MUX_NODE, BRIDGE_NODE = "cmd_vel_mux", "fc_drive_bridge"


# ---------------------------------------------------------------- pure helpers (tests/test_rosq.py)
def dds_participant_ids(proc_net_udp):
    """Participant ids from /proc/net/udp text: Fast DDS binds UDP 7000+id per participant (host-wide)."""
    ids = set()
    for line in proc_net_udp.splitlines()[1:]:
        f = line.split()
        if len(f) < 4 or ":" not in f[1]:
            continue
        try:
            port = int(f[1].rsplit(":", 1)[1], 16)
        except ValueError:
            continue
        if 7000 <= port < 7000 + 250:
            ids.add(port - 7000)
    return sorted(ids)


def dds_summary(ids):
    out = {"participants": len(ids), "max_id": max(ids) if ids else None, "warn": None}
    if ids and max(ids) >= MAX_INITIAL_PEERS - 4:
        out["warn"] = ("DDS participant id %d (limit %d for localhost discovery): too many ROS processes; "
                       "late ones may not discover each other" % (max(ids), MAX_INITIAL_PEERS))
    return out


def topology(cv, cs, cp, tp, topic):
    """Problems with the velocity chain from publisher/subscriber node-name lists (sorted)."""
    problems = []
    if cv != [MUX_NODE]:
        problems.append("/cmd_vel publishers %s (want exactly %s)" % (cv, MUX_NODE))
    if MUX_NODE not in cs:
        problems.append("%s not subscribed to %s" % (MUX_NODE, topic))
    if [n for n in cp if n != BRIDGE_NODE]:
        problems.append("%s publishers %s (want only %s)" % (topic, cp, BRIDGE_NODE))
    if len(tp) > 1:
        problems.append("/teleop_cmd_vel publishers %s (want at most one)" % tp)
    return problems


def complete(cv, cs):
    """The part of the topology that must be discovered before the probe may stop early."""
    return MUX_NODE in cv and MUX_NODE in cs


class Settle:
    """Stop when complete and unchanged for quiet_s (after min_s), or at max_s."""

    def __init__(self, min_s=MIN_S, quiet_s=QUIET_S, max_s=MAX_S):
        self.min_s, self.quiet_s, self.max_s = min_s, quiet_s, max_s
        self.snap, self.since = None, 0.0

    def update(self, t, snap, is_complete):
        if snap != self.snap:
            self.snap, self.since = snap, t
        if t >= self.max_s:
            return True
        return t >= self.min_s and is_complete and t - self.since >= self.quiet_s


def missing_mux_reason(cv, mux_running, waited):
    if MUX_NODE in cv:
        return None
    if mux_running is False:
        return "no cmd_vel_mux process running"
    return ("cmd_vel_mux process running but not discovered after %.1f s (DDS discovery incomplete: "
            "retry; if it persists see fcctl.sh status dds)" % waited)


def mux_process_running(proc="/proc"):
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit()]
    except OSError:
        return None
    for p in pids:
        try:
            with open(os.path.join(proc, p, "cmdline"), "rb") as f:
                argv0 = f.read().split(b"\0", 1)[0]
        except OSError:
            continue
        if os.path.basename(argv0) == b"cmd_vel_mux_node":
            return True
    return False


def read_dds():
    try:
        with open("/proc/net/udp") as f:
            return dds_summary(dds_participant_ids(f.read()))
    except OSError:
        return None


# ---------------------------------------------------------------- rclpy
def main(argv):
    import rclpy
    from controller_manager_msgs.srv import ListControllers, SwitchController
    rclpy.init()
    node = rclpy.create_node("fcctl_query")

    def call(srv, name, req, wait=3.0):
        cli = node.create_client(srv, name)
        if not cli.wait_for_service(timeout_sec=wait):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=wait)
        return fut.result()

    def controllers():
        res = call(ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
        return None if res is None else {c.name: c.state for c in res.controller}

    out = {}
    cmd = argv[0] if argv else "probe"
    if cmd == "probe":
        topic = argv[1] if len(argv) > 1 else "/llm_cmd_vel"

        def names(infos):
            return sorted(i.node_name for i in infos)

        settle, t0 = Settle(), time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            nodes = sorted({n for n, _ in node.get_node_names_and_namespaces()} - {"fcctl_query"})
            cv = names(node.get_publishers_info_by_topic("/cmd_vel"))
            cs = names(node.get_subscriptions_info_by_topic(topic))
            cp = names(node.get_publishers_info_by_topic(topic))
            tp = names(node.get_publishers_info_by_topic("/teleop_cmd_vel"))
            waited = time.monotonic() - t0
            if settle.update(waited, (tuple(nodes), tuple(cv), tuple(cs), tuple(cp), tuple(tp)), complete(cv, cs)):
                break
        out["nodes"] = nodes
        problems = topology(cv, cs, cp, tp, topic)
        dds = read_dds()
        why = missing_mux_reason(cv, mux_process_running() if MUX_NODE not in cv else True, waited)
        if why:
            problems.insert(0, why + (("; " + dds["warn"]) if dds and dds.get("warn") else ""))
        out["mux"] = {"ok": not problems, "alive": MUX_NODE in cv, "problems": problems,
                      "cmd_vel_pubs": cv, "topic": topic, "topic_subs": cs, "topic_pubs": cp, "teleop_pubs": tp}
        out["discovery"] = {"waited_s": round(waited, 1), "settled": complete(cv, cs) and waited < settle.max_s,
                            "nodes": len(nodes)}
        out["dds"] = dds
        out["controllers"] = controllers()
    elif cmd == "controllers":
        out["controllers"] = controllers()
        out["ok"] = out["controllers"] is not None
    elif cmd == "switch":
        want_on = argv[1].split() if len(argv) > 1 else []
        want_off = argv[2].split() if len(argv) > 2 else []
        states = controllers()
        if states is None:
            out = {"ok": False, "error": "controller_manager not reachable"}
        else:
            req = SwitchController.Request()
            req.activate_controllers = [c for c in want_on if states.get(c) != "active"]
            req.deactivate_controllers = [c for c in want_off if states.get(c) == "active" and c not in want_on]
            req.strictness = SwitchController.Request.BEST_EFFORT
            if not req.activate_controllers and not req.deactivate_controllers:
                out = {"ok": True, "changed": False}
            else:
                res = call(SwitchController, "/controller_manager/switch_controller", req, wait=5.0)
                out = {"ok": bool(res and res.ok), "activate": list(req.activate_controllers),
                       "deactivate": list(req.deactivate_controllers),
                       "error": None if res else "no answer (gateway latched or manager busy)"}
                if res is not None and not res.ok:
                    out["error"] = "switch_controller answered ok=false"
            out["controllers"] = controllers()
    else:
        print("usage: rosq.py probe [TOPIC] | controllers | switch '<on...>' '<off...>'", file=sys.stderr)
        return 2
    print(json.dumps(out, separators=(",", ":")))   # compact: fcctl.sh matches '"ok":true'
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
