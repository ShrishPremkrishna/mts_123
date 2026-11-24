"""fc.ctl: terse command line for a running fc app (overseer agent, batch floor sessions). Stdlib only.

    cd final_project && python -m fc.ctl [--url http://127.0.0.1:8800] <command> ...

    status                                  one header line + one line per active robot
    check <robot> [--ball] [--yaw DEG]      RUNBOOK gates (seen, eye fps/calib, auth, RTT, loss, not halted)
    ready <robot> [--preset NAME] [--home]  fcctl ready (stands; re-arms an e-stop). --home: allowed to HOME the legs
    down <robot> [--keep-stack]             fcctl down (lie down; --keep-stack keeps it homed)
    estop <robot|all>                       link E-STOP, waits for the robot's confirmation (3.5 s)
    pi <robot>                              fcctl status: Pi temp/throttle/load, phase, bridge, latch, lean;
                                            2nd line: CPU governor/MHz, fan, top 5 processes by CPU
                                            (fails at >= 80 C or no answer; warns at >= 75 C)
    preset <name|base>                      load a preset (base = params.json + defaults)
    set key=val ...                         live params
    drive <robot> vx vy wz secs             timed teleop pulse in m/s and rad/s (clipped to max_*), then stop
    stop                                    zero speed to every robot (ends the running test)
    test <name> [k=v ...] [--wait]          step vx= vy= wz= seconds= | goto x= y= [yaw=DEG] | deadman [vx= mute_s=]
                                            | push_ball | goalie | match | sim_eval args="..." | imu_test seconds= ...
                                            (all take robot=, sim tests scenario=)
    last [n]                                summaries of the last n runs (default 5)
    batch <file> [--from N] [--no-pause [--max-pause S]] [--stop-at-pause]

Batch files: one command per line as typed after `python -m fc.ctl`; `#` comments; `sleep N`; `pause [N] msg`.
A pause waits for Enter on a terminal; with --no-pause it sleeps N s; otherwise (no terminal, or --stop-at-pause)
the batch exits 4 and prints the --from line to resume. Tests in a batch always wait. The batch stops at the first
failure (exit 1) or health stop (exit 3: e-stop, IMU bad, mux lost, edge, operator STOP, loop error,
2 lost_tag runs in a row).

Exit codes: 0 ok, 1 failed, 2 usage, 3 health stop, 4 paused, 130 interrupted (a running test gets STOP).
In sim without --dry-run-ops robot ops are skipped (they would ssh the real robot); ready clears the sim e-stop.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import sys
import time
import urllib.error
import urllib.request

DEFAULT_URL = "http://127.0.0.1:8800"
# Test results that end a batch: something stopped the robot, or the operator did.
HEALTH_RESULTS = {"estop", "estopped", "imu_bad", "mux_lost", "loop_error", "edge", "stopped", "replaced"}
# ...except deadman's own "stopped" (= came to rest after the mute; it carries stop_time_s), see is_health_stop.
# lost_tag (camera lost the robot > stale_stop_s; the app already zeroed it) ends a batch only twice in a row.
LOST_TAG_STREAK = 2
TEST_KEYS = {"step": {"vx", "vy", "wz", "seconds"}, "deadman": {"vx", "mute_s"}, "goto": {"x", "y", "yaw", "target"},
             "sim_eval": {"args"}, "push_ball": set(), "goalie": set(), "match": set()}
COMMON_KEYS = {"robot", "scenario"}
IMU_KEYS = {"robot", "seconds", "mode", "condition"}
ALIASES = {"push": "push_ball", "imu": "imu_test"}
READY_PHASES = {"homed", "standing", "ready"}     # stack running: ready stands/re-arms, never homes
POLL_S = 0.25
SEEN_TESTS = {"step", "deadman", "goto", "push_ball", "goalie"}   # wait for a fresh observation before these
SEEN_WAIT_S, SEEN_MAX_AGE_S = 3.0, 0.3


class Fail(Exception):
    code = 1


class HealthStop(Fail):
    code = 3


class Paused(Exception):
    code = 4


class Usage(Exception):
    code = 2


# ---------------------------------------------------------------------------------------
# small formatting helpers (pure)
# ---------------------------------------------------------------------------------------
def num(v, nd: int = 2) -> str:
    """Compact number: None -> '-', floats rounded, trailing zeros trimmed."""
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return str(v)


def deg(rad) -> str:
    return "-" if rad is None else f"{math.degrees(rad):.0f}"


def pose_str(p) -> str:
    """[x, y, yaw_rad] or {x, y, yaw} -> (x,y,yaw deg)."""
    if not p:
        return "-"
    x, y, yaw = (p.get("x"), p.get("y"), p.get("yaw")) if isinstance(p, dict) else (list(p) + [None] * 3)[:3]
    return f"({num(x)},{num(y)},{deg(yaw)}°)"


def short(s, n: int = 120) -> str:
    s = " ".join(str(s).split())
    return s if len(s) <= n else s[: n - 3] + "..."


def parse_value(s: str):
    """k=v value: JSON (numbers, true/false, null, lists) else the raw string."""
    try:
        return json.loads(s)
    except ValueError:
        return s


def parse_kv(items: list[str]) -> dict:
    out = {}
    for it in items:
        k, sep, v = it.partition("=")
        if not sep or not k:
            raise Usage(f"expected key=value, got {it!r}")
        out[k] = parse_value(v)
    return out


def test_name(name: str) -> str:
    return ALIASES.get(name, name)


def test_body(name: str, kv: dict) -> dict:
    """ctl test arguments -> the app's /action body. Unknown keys are refused (typos fail early)."""
    name = test_name(name)
    if name not in TEST_KEYS:
        raise Usage(f"unknown test {name!r}; one of {sorted(TEST_KEYS) + ['imu_test']}")
    bad = set(kv) - TEST_KEYS[name] - COMMON_KEYS
    if bad:
        raise Usage(f"{name} does not take {sorted(bad)}; takes {sorted(TEST_KEYS[name] | COMMON_KEYS)}")
    body = {"action": "test", "test": name, **{k: v for k, v in kv.items() if k not in ("x", "y", "yaw", "target")}}
    if name == "goto":
        if "target" in kv:
            t = kv["target"]
            tgt = [parse_value(v) for v in t.split(",")] if isinstance(t, str) else t
        elif "x" in kv and "y" in kv:
            tgt = [kv["x"], kv["y"]]
            if kv.get("yaw") is not None:
                if not isinstance(kv["yaw"], (int, float)):
                    raise Usage("goto yaw is a number in degrees")
                tgt.append(round(math.radians(kv["yaw"]), 4))
        else:
            raise Usage("goto needs x= y= [yaw=DEG] (or target=x,y[,yaw_rad])")
        body["target"] = tgt
    if name == "sim_eval" and "args" in kv:
        body["args"] = str(kv["args"])
    return body


def is_health_stop(s: dict | None) -> bool:
    """A test result that should end a batch. The app names two different things "stopped": the operator's
    STOP (any test) and a deadman run whose robot came to rest (has stop_time_s): only the first is a stop."""
    if not s:
        return True
    r = s.get("result")
    if r == "stopped" and "stop_time_s" in s:
        return False
    return r in HEALTH_RESULTS


def fmt_pass(p) -> str:
    if isinstance(p, dict):
        return " ".join(f"{k}={'-' if v is None else 'ok' if v else 'FAIL'}" for k, v in p.items())
    return "-" if p is None else ("ok" if p else "FAIL")


def run_test_name(s: dict, run: str | None) -> str:
    if s.get("test"):
        return s["test"]
    if not run or "-" not in run:
        return "?"
    name = run.split("-", 1)[1]           # <UTC>-<test>, plus -N when two runs start in the same second
    base, _, n = name.rpartition("-")
    return base if base and n.isdigit() else name


def key_line(test: str, s: dict) -> str | None:
    """The test's own numbers, one line."""
    g = s.get
    if test == "step":
        return (f"cmd {g('cmd')} gain_lin {num(g('gain_lin'))} gain_wz {num(g('gain_wz'))} latency {num(g('latency_s'))}s"
                f" onset {num(g('onset_s'))}s bias {num(g('heading_bias_deg'), 1)}° lateral {num(g('lateral_m'))}m"
                f" drift {num(g('yaw_drift_deg_per_m'), 1)}°/m dist {num(g('dist_m'))}/{num(g('cmd_dist_m'))}m"
                f" pass {fmt_pass(g('pass'))}")
    if test == "deadman":
        return (f"stop_time {num(g('stop_time_s'))}s coast {num(g('coast_m'), 3)}m speed_before {num(g('speed_before'))}"
                f" vx {num(g('vx'))} mute {num(g('mute_s'))}s pass {fmt_pass(g('pass'))}"
                + (" (sim emulated ttl)" if g("emulated") else ""))
    if test == "goto":
        return (f"target {g('target')} final {pose_str(g('final'))} pos_err {num(g('pos_err_m'), 3)}m"
                f" heading_err {num(g('heading_err_deg'), 1)}° pass {fmt_pass(g('pass'))}")
    if test == "push_ball":
        b = g("ball")
        return f"ball at end ({num(b.get('x'))},{num(b.get('y'))})" if b else "ball at end -"
    if test == "goalie":
        return f"conceded {num(g('conceded'))} saved {num(g('saved'))} shots {num(g('shots'))}"
    if test == "match":
        sc = g("score") or {}
        score = " - ".join(f"{k} {v}" for k, v in sc.items()) if isinstance(sc, dict) else num(sc)
        return f"score {score or '-'} touches {num(g('touches'))} goals {short(g('goals'), 60) if g('goals') else '-'}"
    if test == "sim_eval":
        tail = [x for x in str(g("output_tail") or "").splitlines() if x.strip()][-3:]
        return f"rc {num(g('returncode'))}" + ("".join(f"\n  | {short(x, 150)}" for x in tail))
    return None


def robot_line(rid: str, m: dict) -> str:
    return (f"{rid}: seen {num(m.get('seen_frac'))} path {num(m.get('path_m'))}m stale {num(m.get('stale_stops'))}"
            f" rev {num(m.get('wz_reversals'))}/{num(m.get('fast_reversals'))}fast spin {num(m.get('spin_frac'))}"
            f" imu {m.get('imu_worst') or '-'} loss {num(m.get('loss'))} rtt95 {num(m.get('rtt_p95_ms'), 1)}ms"
            f" final {pose_str(m.get('final'))}")


def fmt_summary(s: dict | None, run: str | None = None, run_dir: str | None = None, compact: bool = False) -> list[str]:
    """summary.json (or the app's last_summary) -> at most 6 short lines (2 with compact)."""
    run = run or (s or {}).get("run")
    if not s:
        return [f"{run or '?'}: no summary (still running, or the app was stopped)"]
    test = run_test_name(s, run)
    m = s.get("metrics") or {}
    head = (f"{str(s.get('result', '?')).upper()} {test}{' ' + s['robot'] if s.get('robot') else ''}"
            f" {num(s.get('time_s', m.get('time_s')), 1)}s preset {s.get('preset') or 'base'} · {run}")
    lines = [head]
    k = key_line(test, s)
    if k:
        lines.append(k)
    if compact:
        return [lines[0]] + ([lines[1].splitlines()[0]] if len(lines) > 1 else [])
    robots = m.get("robots") or {}
    who = [s["robot"]] if s.get("robot") in robots else [r for r, v in robots.items() if v.get("seen_frac")]
    if test == "sim_eval":
        who = []
    for rid in who[:2]:
        lines.append(robot_line(rid, robots[rid]))
    probs = []
    for key in ("estop", "imu", "hold", "error"):
        if s.get(key):
            probs.append(f"{key} {short(json.dumps(s[key]) if not isinstance(s[key], str) else s[key], 100)}")
    v = s.get("video") or {}
    if v.get("error"):
        probs.append(f"video {short(v['error'], 60)}")
    if m.get("loop_overruns"):
        probs.append(f"loop overruns {m['loop_overruns']}")
    if probs:
        lines.append("! " + "; ".join(probs))
    lines.append(f"dir {run_dir or 'fc/runs/' + str(run)}")
    return lines[:6]


def robot_status_line(rid: str, st: dict) -> list[str]:
    play = ((st.get("play") or {}).get("robots") or {}).get(rid) or {}
    h = play.get("health") or {}
    L = (st.get("link") or {}).get(rid)
    world = st.get("world") or {}
    me = (world.get("robots") or {}).get(rid)
    stk = (st.get("stack") or {}).get(rid) or {}
    est = (st.get("estop") or {}).get(rid)
    op = (st.get("ops") or {}).get(rid)
    parts = [f"{rid}: {str(h.get('level', '?')).upper()} {h.get('text', '')!r}", f"phase {play.get('phase') or '-'}"]
    if st.get("link") is None:
        parts.append("link " + ("sim" if st.get("source") == "sim" else "off"))
    elif not L:
        parts.append("link none")
    else:
        conn = "" if L.get("connected") else " NOT-CONNECTED"
        parts.append(f"link {L.get('auth')}{conn} p95 {num(L.get('rtt_p95_ms'), 1)}ms loss {num(L.get('loss'))}"
                     f" ack {num(L.get('ack_age'), 1)}s")
        imu = L.get("imu") or {}
        parts.append(f"imu {imu.get('level') or '-'} {num(imu.get('tilt_deg'), 1)}°")
        if L.get("hold"):
            parts.append(f"hold {L['hold']}")
    if est:
        parts.append(f"ESTOPPED({'confirmed' if est.get('confirmed') else 'UNCONFIRMED'} by {est.get('by')})")
    for key, txt in (("imu_bad", "IMU-BAD"), ("mux_lost", "MUX-LOST")):
        if rid in (st.get(key) or []):
            parts.append(txt)
    parts.append(f"cap {num(stk.get('speed_cap'))} hh {stk.get('heading_hold') or '-'}")
    parts.append(f"pose {pose_str(me)} {num(me.get('age'), 1)}s" if me else "pose NOT SEEN")
    if op:
        rc = "running" if op.get("running") else f"rc {op.get('rc')}"
        parts.append(f"op {op.get('op')} {rc}")
    out = [" · ".join(parts)]
    for w in stk.get("warn") or []:
        out.append(f"  warn: {short(w, 140)}")
    return out


def fmt_status(st: dict) -> list[str]:
    t = st.get("test")
    tdesc = "-"
    if t:
        tdesc = f"{t.get('name')}{'(' + t['robot'] + ')' if t.get('robot') else ''} {num(t.get('elapsed'), 1)}s" \
                + (f" {t['phase']}" if t.get("phase") else "")
    pr = st.get("preset") or {}
    pname = (pr.get("name") or "base") + ("*" if pr.get("modified") else "")
    stack = " ".join(f"{k}={v}" for k, v in (pr.get("stack") or {}).items())
    lp = st.get("loop") or {}
    ball = (st.get("world") or {}).get("ball")
    bdesc = f"ball ({num(ball.get('x'))},{num(ball.get('y'))}) {num(ball.get('age'), 1)}s" if ball else "ball NOT SEEN"
    head = [f"{st.get('source')}{' dry-run-ops' if st.get('dry_run_ops') else ''}", f"mode {st.get('mode')}",
            f"test {tdesc}", f"preset {pname}" + (f" [{stack}]" if stack else ""),
            f"loop {num(lp.get('hz'), 1)}Hz ovr {lp.get('overruns')}", bdesc]
    lines = [" · ".join(head)]
    eye = st.get("eye")
    if eye:
        lines.append(f"eye {num(eye.get('fps'), 1)}fps calib {'ok' if eye.get('calibrated') else 'NO'}"
                     f" err {num(eye.get('calib_err_px'))}px · {short(eye.get('message', ''), 80)}")
    for rid in st.get("active") or []:
        lines += robot_status_line(rid, st)
    for b in ((st.get("play") or {}).get("banners") or []):
        if b.get("kind") in ("estop", "down"):
            lines.append(f"  banner: {short(b.get('text'), 140)}")
    return lines


PI_WARN_C, PI_STOP_C = 75.0, 80.0    # RUNBOOK aims < 75 C; fcctl boot refuses to home at 80 C


def pi_line(rid: str, st: dict) -> tuple[str, str | None]:
    """fcctl status JSON -> (one line, failure reason or None). Pure."""
    t = st.get("temp_c")
    thr = st.get("throttled")
    flag = " HOT" if isinstance(t, (int, float)) and t >= PI_WARN_C else ""
    thr_txt = f" throttled {thr}" + (" (!)" if thr not in (None, "0x0") else "") if thr is not None else ""
    load = st.get("load") or []
    br = (st.get("bridge") or {}).get("alive")
    parts = [f"PI {rid}: {num(t, 1)} C{flag}{thr_txt}", f"load {num(load[0], 2) if load else '-'}",
             f"phase {st.get('phase') or '-'}", f"bridge {'up' if br else 'down' if br is False else '-'}",
             f"cap {num(st.get('speed_cap'))} liedown {st.get('liedown')}"]
    if st.get("lean") is not None:
        parts.append(f"lean {st.get('lean')} camera {'on' if st.get('camera') else 'off'}")
    if st.get("estopped"):
        parts.append(f"ESTOPPED({(st.get('estop') or {}).get('reason')})")
    why = None
    if not isinstance(t, (int, float)):
        why = "no temperature in fcctl status"
    elif t >= PI_STOP_C:
        why = f"Pi at {t:g} C (>= {PI_STOP_C:g}): let it cool"
    return " · ".join(parts), why


def pi_heat_line(st: dict) -> str | None:
    """fcctl status "heat" (robot/heat.py) -> one line: governor/MHz, fan, top processes. Pure."""
    h = st.get("heat")
    if not isinstance(h, dict):
        return None
    cpu, fan = h.get("cpu") or {}, h.get("fan") or {}
    parts = [f"cpu {cpu.get('governor') or '-'} {num(cpu.get('cur_mhz'))}/{num(cpu.get('max_mhz'))} MHz"]
    if fan:
        parts.append(f"fan {num(fan.get('state'))}/{num(fan.get('max_state'))} {num(fan.get('rpm'))} rpm")
    top = ", ".join(f"{p.get('cmd')} {num(p.get('pct'), 1)}%" for p in (h.get("top") or [])[:5])
    parts.append(f"top {top or '-'}")
    return "  " + " · ".join(parts)


def progress_line(obj: dict) -> str:
    el = obj.get("elapsed")
    el = f"{el:6.1f}s" if isinstance(el, (int, float)) else "      -"
    bad = " FAIL" if obj.get("ok") is False else ""
    extra = "".join(f" [{k} {short(obj[k], 60)}]" for k in ("action", "at", "next", "speed_cap", "heading_hold", "stack")
                    if obj.get(k) not in (None, ""))
    return f"  {el} {obj.get('phase', '?')}{bad}: {short(obj.get('msg') or obj.get('error') or '', 140)}{extra}"


def json_lines(text: str) -> list[tuple[str, dict]]:
    out = []
    for raw in (text or "").splitlines():
        try:
            obj = json.loads(raw)
        except ValueError:
            continue
        if isinstance(obj, dict):
            out.append((raw, obj))
    return out


def parse_pause(argv: list[str]) -> tuple[float, str]:
    """pause [N] message -> (N seconds for --no-pause, message)."""
    secs = 0.0
    if argv and _is_num(argv[0]):
        secs, argv = float(argv[0]), argv[1:]
    return secs, " ".join(argv) or "paused"


def _is_num(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def parse_batch(text: str) -> list[tuple[int, list[str]]]:
    out = []
    for i, line in enumerate(text.splitlines(), 1):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        try:
            argv = shlex.split(s, comments=True)
        except ValueError as e:
            raise Usage(f"line {i}: {e}")
        if argv:
            out.append((i, argv))
    return out


def check_robot(st: dict, rid: str, ball: bool = False, yaw: float | None = None, fps: float = 15.0,
                calib: float = 1.5, rtt: float = 20.0, loss: float = 0.0, yaw_tol: float = 20.0) -> tuple[bool, list[str]]:
    """RUNBOOK setup gates -> (ok, notes). Camera/link gates apply only when that part exists."""
    bad, good = [], []
    if rid not in (st.get("active") or []):
        return False, [f"{rid} not active ({st.get('active')})"]
    if rid in (st.get("estop") or {}):
        bad.append("E-STOPPED (ready re-arms)")
    for key in ("imu_bad", "mux_lost"):
        if rid in (st.get(key) or []):
            bad.append(key)
    w = st.get("world") or {}
    me = (w.get("robots") or {}).get(rid)
    stale = float((st.get("params") or {}).get("stale_stop_s", 0.5))
    if not me or (me.get("age") or 0) > stale:
        bad.append("robot not seen by the camera")
    else:
        good.append(f"seen {num(me.get('age'), 2)}s")
        if yaw is not None and st.get("source") == "sim":   # the yaw gate checks the camera's tag yaw vs a placed robot
            good.append(f"yaw {deg(me.get('yaw'))}° (sim: not gated)")
        elif yaw is not None:
            err = (math.degrees(me.get("yaw") or 0) - yaw + 180) % 360 - 180
            (good if abs(err) <= yaw_tol else bad).append(f"yaw {deg(me.get('yaw'))}° (want {num(yaw)}±{num(yaw_tol)})")
    if ball:
        b = w.get("ball")
        (good if b and (b.get("age") or 0) <= 1.0 else bad).append(
            f"ball ({num(b.get('x'))},{num(b.get('y'))})" if b else "ball not seen")
    eye = st.get("eye")
    if eye:
        (good if (eye.get("fps") or 0) >= fps else bad).append(f"eye {num(eye.get('fps'), 1)}fps (min {num(fps)})")
        ce = eye.get("calib_err_px")
        if not eye.get("calibrated"):
            bad.append("camera not calibrated (corner tags?)")
        else:
            (good if ce is not None and ce < calib else bad).append(f"calib {num(ce)}px (max {num(calib)})")
    if st.get("link") is not None:
        L = (st.get("link") or {}).get(rid) or {}
        (good if L.get("auth") == "ok" else bad).append(f"auth {L.get('auth')}")
        (good if L.get("connected") else bad).append("connected" if L.get("connected") else "NOT connected (bridge up?)")
        p95 = L.get("rtt_p95_ms")
        (good if p95 is not None and p95 < rtt else bad).append(f"p95 {num(p95, 1)}ms (max {num(rtt)})")
        lo = L.get("loss")
        (good if lo is not None and lo <= loss else bad).append(f"loss {num(lo)} (max {num(loss)})")
        lvl = (L.get("imu") or {}).get("level")
        (bad if lvl == "bad" else good).append(f"imu {lvl or '-'}")
    return not bad, (["bad: " + "; ".join(bad)] if bad else []) + (["ok: " + ", ".join(good)] if good else [])


# ---------------------------------------------------------------------------------------
# HTTP client
# ---------------------------------------------------------------------------------------
class Client:
    def __init__(self, url: str = DEFAULT_URL, timeout: float = 8.0):
        self.url, self.timeout = url.rstrip("/"), timeout

    def _open(self, req):
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            try:
                return json.loads(e.read())
            except ValueError:
                raise Fail(f"HTTP {e.code} from {req.full_url}")
        except (urllib.error.URLError, OSError) as e:
            raise Fail(f"app not reachable at {self.url} ({getattr(e, 'reason', e)}): start it with fc/run.sh")

    def get(self, path: str):
        return self._open(urllib.request.Request(self.url + path))

    def post(self, path: str, body: dict) -> dict:
        req = urllib.request.Request(self.url + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
        res = self._open(req)
        return res if isinstance(res, dict) else {"ok": False, "error": f"bad reply {res!r}"}

    def state(self) -> dict:
        return self.get("/state")


# ---------------------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------------------
def say(*lines: str) -> None:
    for x in lines:
        print(x, flush=True)


def need_ok(res: dict, what: str) -> dict:
    if not res.get("ok"):
        raise Fail(f"{what}: {res.get('error') or res.get('errors') or res}")
    return res


def cmd_status(c: Client, a) -> None:
    say(*fmt_status(c.state()))


def cmd_check(c: Client, a) -> None:
    ok, notes = check_robot(c.state(), a.robot, ball=a.ball, yaw=a.yaw, fps=a.fps, calib=a.calib, rtt=a.rtt,
                            loss=a.loss)
    say(f"CHECK {a.robot} {'PASS' if ok else 'FAIL'}: " + " | ".join(notes))
    if not ok:
        raise Fail(f"check {a.robot} failed")


def cmd_preset(c: Client, a) -> None:
    load_preset(c, a.name)


def load_preset(c: Client, name: str) -> None:
    res = c.post("/preset", {"op": "base"} if name == "base" else {"op": "load", "name": name})
    need_ok(res, f"preset {name}")
    if name == "base":
        say("preset base (params.json + defaults)")
        return
    stack = " ".join(f"{k}={v}" for k, v in (res.get("stack") or {}).items())
    say(f"preset {name}: {res.get('applied')} keys" + (f", stack {stack}" if stack else ""))
    if res.get("errors"):
        raise Fail(f"preset {name} loaded with rejected keys: {'; '.join(res['errors'])}")


def cmd_set(c: Client, a) -> None:
    kv = parse_kv(a.pairs)
    if not kv:
        raise Usage("set key=val ...")
    res = c.post("/params", kv)
    applied = " ".join(f"{k}={v}" for k, v in (res.get("applied") or {}).items())
    say(f"set {applied or '(nothing)'}" + (f"; rejected: {'; '.join(res['errors'])}" if res.get("errors") else ""))
    if res.get("errors"):
        raise Fail("some params were rejected")


def cmd_stop(c: Client, a) -> None:
    need_ok(c.post("/action", {"action": "stop"}), "stop")
    say("STOP sent: zero speed to every robot")


def cmd_estop(c: Client, a) -> None:
    target = None if a.robot == "all" else a.robot
    res = need_ok(c.post("/action", {"action": "estop", "robot": target}), "estop")
    robots = res.get("robots") or []
    end, st = time.monotonic() + 3.5, {}
    while True:
        st = c.state()
        es = st.get("estop") or {}
        if all((es.get(r) or {}).get("confirmed") for r in robots) or time.monotonic() > end:
            break
        time.sleep(0.1)
    es, bad = st.get("estop") or {}, []
    for r in robots:
        e = es.get(r) or {}
        ok = bool(e.get("confirmed"))
        say(f"E-STOP {r}: {'confirmed' if ok else 'NOT CONFIRMED'} · {short(e.get('text', ''), 120)}")
        if not ok:
            bad.append(r)
    if bad:
        raise Fail(f"E-STOP not confirmed by {', '.join(bad)}: USE R3")


def cmd_pi(c: Client, a) -> None:
    rid = a.robot
    mode = robot_ops_mode(c.state())
    if mode == "sim":
        say(f"sim: no robot ops (app without --dry-run-ops); pi {rid} skipped")
        return
    op = run_op(c, rid, {"op": "status"}, 90)
    if is_dry(op):
        say(f"PI {rid} (dry run: nothing ran)")
        return
    st = next((o for _, o in reversed(json_lines(op.get("out", ""))) if "phase" in o), None)
    if op.get("rc") != 0 or st is None:
        raise Fail(f"pi {rid}: fcctl status failed (rc {op.get('rc')}): {op_error(op)}")
    line, why = pi_line(rid, st)
    say(line)
    heat = pi_heat_line(st)
    if heat:
        say(heat)
    if why:
        raise Fail(f"pi {rid}: {why}")


def robot_ops_mode(st: dict) -> str:
    """'real' | 'dry' | 'sim' (sim without --dry-run-ops: ops would ssh the real robot, so ctl skips them)."""
    if st.get("dry_run_ops"):
        return "dry"
    return "sim" if st.get("source") == "sim" else "real"


def run_op(c: Client, rid: str, body: dict, timeout: float, stream: bool = False) -> dict:
    """POST /robot, then follow ops[rid] until it ends. Streams fcctl JSON progress lines when asked."""
    res = need_ok(c.post("/robot", {"robot": rid, **body}), f"{body.get('op')} {rid}")
    cmd, printed, end = res.get("cmd"), set(), time.monotonic() + timeout
    while True:
        op = (c.state().get("ops") or {}).get(rid)
        if not op or op.get("cmd") != cmd:
            raise Fail(f"{body.get('op')} {rid}: replaced by another op ({(op or {}).get('op')})")
        if stream:
            for raw, obj in json_lines(op.get("out", "")):
                if raw not in printed:
                    printed.add(raw)
                    say(progress_line(obj))
        if not op.get("running"):
            return op
        if time.monotonic() > end:
            raise Fail(f"{body.get('op')} {rid}: no end after {timeout:.0f} s (still running on the app; see status)")
        time.sleep(0.3)


def op_error(op: dict) -> str:
    prog = op.get("progress") or next((o for _, o in reversed(json_lines(op.get("out", ""))) if o.get("ok") is False), {})
    err = prog.get("error") or prog.get("msg")
    if not err:
        plain = [x for x in (op.get("out") or "").splitlines() if x.strip() and not x.lstrip().startswith("{")]
        err = plain[-1] if plain else f"rc {op.get('rc')}"
    return short(err, 200)


def is_dry(op: dict) -> bool:
    return (op.get("out") or "").lstrip().startswith("DRY-RUN") or (op.get("cmd") or "").startswith("echo DRY-RUN")


def cmd_ready(c: Client, a) -> None:
    rid = a.robot
    if a.preset:
        load_preset(c, a.preset)
    st = c.state()
    mode = robot_ops_mode(st)
    if mode == "sim":
        if rid in (st.get("estop") or {}):
            need_ok(c.post("/action", {"action": "estop_clear", "robot": rid}), "estop_clear")
            say(f"sim: no robot ops (app without --dry-run-ops); e-stop latch on {rid} cleared (stands in for Ready)")
        else:
            say(f"sim: no robot ops (app without --dry-run-ops); {rid} needs no Ready")
        return
    if mode == "real":
        L = (st.get("link") or {}).get(rid) or {}
        if not L.get("connected"):     # bridge acks = stack running: ready cannot home. Otherwise ask fcctl.
            op = run_op(c, rid, {"op": "status"}, 90)
            # read the phase from the op's own output: the app parses robot_status just after running=False
            phase = next((o.get("phase") for _, o in reversed(json_lines(op.get("out", ""))) if "phase" in o), None)
            say(f"  status: phase {phase or 'unknown'} (rc {op.get('rc')})")
            if phase not in READY_PHASES and not a.home:
                raise Fail(f"{rid} phase {phase or 'unknown'}: ready would START the stack and HOME the legs."
                           " Robot on the stand? Then rerun with --home")
    timeout = 330
    op = run_op(c, rid, {"op": "ready", "confirm": True}, timeout, stream=True)
    if is_dry(op):
        say(f"  dry-run: {short(op.get('out') or op.get('cmd'), 200)}")
        say(f"READY {rid} (dry run: nothing ran)")
        return
    if op.get("rc") != 0:
        raise Fail(f"ready {rid} failed (rc {op.get('rc')}): {op_error(op)}")
    lines = [o for _, o in json_lines(op.get("out", ""))]
    last = lines[-1] if lines else (op.get("progress") or {})
    cap = next((o["speed_cap"] for o in reversed(lines) if "speed_cap" in o), None)
    hh = next((o["heading_hold"] for o in reversed(lines) if "heading_hold" in o), None)
    phase = last.get("phase")
    if phase == "homed":
        say(f"HOMED {rid}: move it to the floor, then `ready {rid}` again")
        return
    want = (c.state().get("preset") or {}).get("stack") or {}
    say(f"{str(phase).upper()} {rid} · cap {num(cap)} hh {hh or '-'}")
    if "speed_cap" in want and cap is not None and abs(float(cap) - float(want["speed_cap"])) > 1e-6:
        raise Fail(f"preset wants speed_cap {want['speed_cap']} but the stack runs {cap}: down, then ready --home")
    if phase != "ready":
        raise Fail(f"ready {rid} ended at phase {phase!r}, not ready")


def cmd_down(c: Client, a) -> None:
    rid = a.robot
    st = c.state()
    if robot_ops_mode(st) == "sim":
        say(f"sim: no robot ops (app without --dry-run-ops); down {rid} skipped")
        return
    body = {"op": "down", **({"keep_stack": True} if a.keep_stack else {})}
    c.post("/action", {"action": "stop"})       # zero speed (ends a running test) before it lies down
    op = run_op(c, rid, body, 120, stream=True)
    if is_dry(op):
        say(f"  dry-run: {short(op.get('out') or op.get('cmd'), 200)}")
        say(f"DOWN {rid} (dry run: nothing ran)")
        return
    if op.get("rc") != 0:
        raise Fail(f"DOWN FAILED {rid} (rc {op.get('rc')}): still standing? use R3 or support the robot: {op_error(op)}")
    last = (json_lines(op.get("out", "")) or [("", {})])[-1][1]
    say(f"DOWN {rid} · stack {last.get('stack') or ('kept' if a.keep_stack else 'stopped')}"
        + (f" · lowering ABORTED by E-STOP ({last['aborted']}): legs off, latched" if last.get("aborted") else ""))
    if last.get("aborted"):   # the operator cut the lie-down short: stop a batch here
        raise HealthStop(f"down {rid}: lowering aborted by E-STOP ({last['aborted']}); Ready --confirm re-arms")


def wait_seen(c: Client, rid: str | None, st: dict | None = None, timeout: float | None = None) -> bool:
    """Wait until the camera sees the robot (fresh observation), so a test does not start inside a tag dropout."""
    end = time.monotonic() + (SEEN_WAIT_S if timeout is None else timeout)
    while True:
        me = (((st or {}).get("world") or {}).get("robots") or {}).get(rid)
        if me and (me.get("age") or 0) <= SEEN_MAX_AGE_S:
            return True
        if time.monotonic() > end:
            return False
        time.sleep(0.1)
        st = c.state()


def wait_summary(c: Client, run: str, st: dict | None = None, wait_s: float = 4.0) -> dict | None:
    """The finished run's summary: the app's last_summary, else summary.json (written on a side thread)."""
    end = time.monotonic() + wait_s
    while True:
        ls = (st or {}).get("last_summary") or {}
        if ls.get("run") == run:
            return ls
        s = c.get(f"/runs/{run}/summary.json")
        if isinstance(s, dict) and s.get("result"):
            return {"run": run, **s}
        if time.monotonic() > end:
            return None
        time.sleep(0.2)
        st = c.state()


def test_timeout(name: str, params: dict) -> float:
    p = lambda k, d: float(params.get(k, d))  # noqa: E731
    if name in ("push_ball", "goalie", "goto"):
        return p("test_timeout_s", 45) + 15
    if name == "match":
        return p("match_s", 300) + 30
    if name == "step":
        return 0.7 + p("step_s", 2) + p("step_settle_s", 1.5) + 15
    if name == "deadman":
        return 0.5 + p("deadman_drive_s", 2) + p("deadman_mute_s", 2) + 15
    return 900


def cmd_test(c: Client, a) -> None:
    name = test_name(a.name)
    kv = parse_kv(a.pairs)
    if name == "imu_test":
        return imu_test(c, kv)
    if name == "drive":
        raise Usage("use `drive <robot> vx vy wz secs`")
    body = test_body(name, kv)
    st0 = c.state()
    rid = body.get("robot") or (st0.get("active") or [None])[0]
    if name in SEEN_TESTS and not wait_seen(c, rid, st0):
        raise Fail(f"test {name}: {rid} not seen by the camera for {SEEN_WAIT_S:.0f} s; not started")
    res = need_ok(c.post("/action", body), f"test {name}")
    run_dir = res.get("run") or ""
    run = os.path.basename(run_dir)
    if not a.wait:
        say(f"started {name} -> {run}")
        return
    end = time.monotonic() + (a.timeout or test_timeout(name, st0.get("params") or {}))
    try:
        while True:
            st = c.state()
            t = st.get("test")
            if not t or t.get("run_dir") != run_dir:
                break
            if time.monotonic() > end:
                c.post("/action", {"action": "stop"})
                raise Fail(f"test {name}: no end after {end - time.monotonic():.0f} s; STOP sent")
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        c.post("/action", {"action": "stop"})
        say(f"interrupted: STOP sent ({run})")
        raise
    s = wait_summary(c, run, st)
    say(*fmt_summary(s, run, run_dir))
    if is_health_stop(s):
        raise HealthStop(f"{name} ended {(s or {}).get('result')}")
    return s


def imu_test(c: Client, kv: dict) -> None:
    bad = set(kv) - IMU_KEYS
    if bad:
        raise Usage(f"imu_test takes {sorted(IMU_KEYS)}")
    st = c.state()
    rid = kv.pop("robot", None) or (st.get("active") or ["blue"])[0]
    if robot_ops_mode(st) == "sim":
        say(f"sim: no robot ops (app without --dry-run-ops); imu_test {rid} skipped")
        return
    secs = int(kv.get("seconds", 5))
    op = run_op(c, rid, {"op": "imu_test", **kv, "seconds": secs}, secs + 120)
    tail = [x for x in (op.get("out") or "").splitlines() if x.strip()][-5:]
    say(f"IMU_TEST {rid} rc {op.get('rc')}", *(f"  | {short(x, 160)}" for x in tail))
    if op.get("rc") != 0:
        raise Fail(f"imu_test {rid} failed (rc {op.get('rc')})")


def cmd_drive(c: Client, a) -> None:
    rid, secs = a.robot, a.secs
    if not 0 < secs <= 10:
        raise Usage("drive secs in (0, 10]")
    p = c.state().get("params") or {}
    frac, clip = [], []
    for v, k in ((a.vx, "max_vx"), (a.vy, "max_vy"), (a.wz, "max_wz")):
        mx = float(p.get(k) or 0)
        f = 0.0 if v == 0 else (max(-1.0, min(1.0, v / mx)) if mx > 0 else 0.0)
        if v and (mx <= 0 or abs(v) > mx + 1e-9):
            clip.append(f"{k} {num(mx)}")
        frac.append(f)
    res = need_ok(c.post("/action", {"action": "test", "test": "drive", "robot": rid}), f"drive {rid}")
    run_dir = res.get("run") or ""
    run, why = os.path.basename(run_dir), None
    try:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            r = c.post("/action", {"action": "teleop", "robot": rid, "vx": frac[0], "vy": frac[1], "wz": frac[2]})
            if not r.get("ok"):
                why = r.get("error")
                break
            time.sleep(0.1)
    finally:
        c.post("/action", {"action": "stop"})
    s = wait_summary(c, run)
    m = (((s or {}).get("metrics") or {}).get("robots") or {}).get(rid) or {}
    result = (s or {}).get("result")
    say(f"DRIVE {rid} vx {num(a.vx)} vy {num(a.vy)} wz {num(a.wz)} {num(secs, 1)}s"
        + (f" (clipped to {', '.join(clip)})" if clip else "")
        + f": path {num(m.get('path_m'))}m final {pose_str(m.get('final'))} seen {num(m.get('seen_frac'))}"
        f" imu {m.get('imu_worst') or '-'} loss {num(m.get('loss'))} · {run}")
    if why:
        if result in HEALTH_RESULTS - {"stopped"}:
            raise HealthStop(f"drive ended {result}: {why}")
        raise Fail(f"drive interrupted: {why} (result {result})")


def cmd_last(c: Client, a) -> None:
    runs = c.get("/runs") or []
    if not runs:
        say("no runs")
    for r in runs[: a.n]:
        say(*fmt_summary(r.get("summary"), r.get("name"), compact=True))


def cmd_sleep(c: Client, a) -> None:
    time.sleep(a.secs)


# ---------------------------------------------------------------------------------------
# parser, batch, main
# ---------------------------------------------------------------------------------------
class Parser(argparse.ArgumentParser):
    def error(self, message):
        raise Usage(f"{self.prog}: {message}")


def build_parser() -> Parser:
    ap = Parser(prog="python -m fc.ctl", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("FC_URL", DEFAULT_URL))
    sub = ap.add_subparsers(dest="cmd", parser_class=Parser)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    p = sub.add_parser("check")
    p.add_argument("robot")
    p.add_argument("--ball", action="store_true", help="also require the ball seen")
    p.add_argument("--yaw", type=float, default=None, help="robot heading in degrees (±20)")
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--calib", type=float, default=1.5)
    p.add_argument("--rtt", type=float, default=20.0)
    p.add_argument("--loss", type=float, default=0.0)
    p.set_defaults(fn=cmd_check)
    p = sub.add_parser("ready")
    p.add_argument("robot")
    p.add_argument("--preset", default=None)
    p.add_argument("--home", action="store_true", help="allowed to start the stack (HOMES the legs: robot on the stand)")
    p.set_defaults(fn=cmd_ready)
    p = sub.add_parser("down")
    p.add_argument("robot")
    p.add_argument("--keep-stack", action="store_true")
    p.set_defaults(fn=cmd_down)
    p = sub.add_parser("estop")
    p.add_argument("robot", help="robot id or all")
    p.set_defaults(fn=cmd_estop)
    p = sub.add_parser("pi")
    p.add_argument("robot")
    p.set_defaults(fn=cmd_pi)
    p = sub.add_parser("preset")
    p.add_argument("name")
    p.set_defaults(fn=cmd_preset)
    p = sub.add_parser("set")
    p.add_argument("pairs", nargs="*")
    p.set_defaults(fn=cmd_set)
    p = sub.add_parser("drive")
    p.add_argument("robot")
    for k in ("vx", "vy", "wz", "secs"):
        p.add_argument(k, type=float)
    p.set_defaults(fn=cmd_drive)
    sub.add_parser("stop").set_defaults(fn=cmd_stop)
    p = sub.add_parser("test")
    p.add_argument("name")
    p.add_argument("pairs", nargs="*")
    p.add_argument("--wait", action="store_true")
    p.add_argument("--timeout", type=float, default=None)
    p.set_defaults(fn=cmd_test)
    p = sub.add_parser("last")
    p.add_argument("n", nargs="?", type=int, default=5)
    p.set_defaults(fn=cmd_last)
    p = sub.add_parser("sleep")
    p.add_argument("secs", type=float)
    p.set_defaults(fn=cmd_sleep)
    p = sub.add_parser("batch")
    p.add_argument("file")
    p.add_argument("--from", dest="start", type=int, default=1, help="first line number to run")
    p.add_argument("--no-pause", action="store_true", help="pause lines sleep their N seconds")
    p.add_argument("--stop-at-pause", action="store_true", help="exit 4 at each pause with the resume line")
    p.add_argument("--max-pause", type=float, default=None, help="with --no-pause: sleep at most this long (sim rehearsal)")
    p.set_defaults(fn=None)
    return ap


def run_batch(c: Client, a, ap: Parser, interactive: bool | None = None) -> None:
    try:
        text = open(a.file).read()
    except OSError as e:
        raise Usage(f"batch: {e}")
    steps = parse_batch(text)
    for lineno, argv in steps:   # parse everything first: a typo on line 80 should not wait 9 minutes
        if argv[0] in ("pause", "batch"):
            if argv[0] == "batch":
                raise Usage(f"line {lineno}: no nested batch")
            continue
        try:
            sub = ap.parse_args(argv)
            if sub.fn is cmd_test:
                n = test_name(sub.name)
                if n not in ("imu_test", "drive"):
                    test_body(n, parse_kv(sub.pairs))
        except Usage as e:
            raise Usage(f"{a.file} line {lineno}: {e}")
    if interactive is None:
        interactive = sys.stdin.isatty() and not a.stop_at_pause
    resume = f"python -m fc.ctl batch {a.file}"
    t0, n, lost = time.monotonic(), 0, 0
    for lineno, argv in steps:
        if lineno < a.start:
            continue
        say(f"[{lineno}] {' '.join(argv)}")
        if argv[0] == "pause":
            secs, msg = parse_pause(argv[1:])
            if a.no_pause:
                if a.max_pause is not None:
                    secs = min(secs, a.max_pause)
                say(f"  PAUSE (no-pause, {num(secs)} s): {msg}")
                time.sleep(secs)
                continue
            if interactive:
                ans = input(f"  PAUSE: {msg}\n  Enter = continue, q = stop: ").strip().lower()
                if ans != "q":
                    continue
            say(f"PAUSED at line {lineno}: {msg}", f"resume: {resume} --from {_next_line(steps, lineno)}")
            raise Paused()
        sub = ap.parse_args(argv)
        if sub.fn is cmd_test:
            sub.wait = True
        try:
            res = sub.fn(c, sub)
            n += 1
            if sub.fn is cmd_test:
                lost = lost + 1 if isinstance(res, dict) and res.get("result") == "lost_tag" else 0
                if lost >= LOST_TAG_STREAK:
                    raise HealthStop(f"lost_tag {lost} tests in a row: the camera is not seeing the robot")
        except (Fail, Usage) as e:
            kind = "HEALTH STOP" if isinstance(e, HealthStop) else "FAILED"
            say(f"BATCH STOPPED at line {lineno} ({kind}): {e}", f"fix it, then resume: {resume} --from {lineno}")
            e._said = True
            raise
    say(f"BATCH DONE {a.file}: {n} commands in {num((time.monotonic() - t0) / 60, 1)} min")


def _next_line(steps, lineno: int) -> int:
    later = [ln for ln, _ in steps if ln > lineno]
    return later[0] if later else lineno + 1


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    ap = build_parser()
    try:
        a = ap.parse_args(argv)
        if not a.cmd:
            ap.print_help()
            return 2
        signal.signal(signal.SIGTERM, _sigterm)   # a killed ctl stops the running test like Ctrl-C does
        c = Client(a.url)
        if a.cmd == "batch":
            run_batch(c, a, ap)
        else:
            a.fn(c, a)
        return 0
    except Usage as e:
        print(f"usage error: {e}", file=sys.stderr)
        return e.code
    except Paused as e:
        return e.code
    except Fail as e:
        if not getattr(e, "_said", False):
            print(f"{'HEALTH STOP' if isinstance(e, HealthStop) else 'FAILED'}: {e}", flush=True)
        return e.code
    except KeyboardInterrupt:
        print("interrupted", flush=True)
        return 130


if __name__ == "__main__":
    sys.exit(main())
