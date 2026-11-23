"""Laptop side of provision.sh: robots.json consistency, probe evaluation, readiness report.

    python3 report.py config  --robots R --robot-id ID --team T --tag N --host H   -> prints ok|fill-host, exit 2 on conflict
    python3 report.py set-host --robots R --robot-id ID --host H                   -> fills an empty host in robots.json
    python3 report.py report  --robots R --robot-id ID --team T --tag N --host H --probe FILE --local-dir DIR
                              [--known-hosts-fp OTHER_ID=FP ...]                  -> report; exit 0 ready, 1 not ready

Stdlib only (runs on the Mac's system python3). Levels: FAIL blocks `fcctl.sh start`, WARN needs a look.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

ID_RE = re.compile(r"^[A-Za-z0-9_.-]{1,32}$")
HOST_RE = re.compile(r"^[A-Za-z0-9_.@:-]{1,253}$")
TEAMS = ("blue", "red")
STOCK_PKGS = ("neural_controller", "control_board_hardware_interface", "cmd_vel_mux", "teleop_twist_joy", "joy",
              "controller_manager")
STAGED = ("drive_bridge.py", "fcctl.sh", "imu_health.py", "profiles/stock.sh")
MTS_POLICY_SHA = "ecf3a4ab"     # default_policy.json on mts (audit 2026-11-23); same in stock and tej_u6
MIN_DISK_GB = 2.0


class ConfigError(Exception):
    pass


# ---- robots.json ------------------------------------------------------------------------------
def check_args(robot_id: str, team: str, tag: int, host: str) -> None:
    if not ID_RE.match(robot_id):
        raise ConfigError(f"bad robot id {robot_id!r} (letters, digits, _ . -; at most 32)")
    if team not in TEAMS:
        raise ConfigError(f"team must be one of {TEAMS}, not {team!r}")
    if not 4 <= tag <= 586:
        raise ConfigError(f"tag {tag}: robot tags are 4..586 (0-3 are the field corners)")
    if not HOST_RE.match(host) or host.startswith("-"):
        raise ConfigError(f"bad ssh host {host!r}")


def config_action(robots: list[dict], robot_id: str, team: str, tag: int, host: str) -> str:
    """'ok' if the robots.json entry already says this; 'fill-host' if only its host is empty.
    Raises ConfigError for anything that would put one robot's identity on another robot."""
    check_args(robot_id, team, tag, host)
    mine = [r for r in robots if r.get("id") == robot_id]
    if not mine:
        snippet = json.dumps({"id": robot_id, "team": team, "tag": tag, "host": host, "profile": "stock",
                              "udp_host": ""})
        raise ConfigError(f"no robots.json entry with id {robot_id!r}; add one first, e.g. {snippet}")
    r = mine[0]
    if r.get("team") != team or int(r.get("tag", -1)) != tag:
        raise ConfigError(f"robots.json says {robot_id} is team {r.get('team')} tag {r.get('tag')}, "
                          f"not team {team} tag {tag}; fix robots.json or the arguments")
    for o in robots:
        if o is r:
            continue
        if o.get("host") and o.get("host") == host:
            raise ConfigError(f"host {host!r} belongs to robot {o.get('id')!r} in robots.json; refusing to "
                              f"provision it as {robot_id!r}")
        if int(o.get("tag", -1)) == tag:
            raise ConfigError(f"tag {tag} is already robot {o.get('id')!r}'s")
        if o.get("team") == team:
            raise ConfigError(f"team {team} is already robot {o.get('id')!r}'s")
    if r.get("host") in ("", None):
        return "fill-host"
    if r["host"] != host:
        raise ConfigError(f"robots.json has {robot_id} at host {r['host']!r}, not {host!r}; edit robots.json "
                          "if the robot really moved")
    return "ok"


def set_host(path: Path, robot_id: str, host: str) -> None:
    robots = json.loads(path.read_text())
    for r in robots:
        if r.get("id") == robot_id:
            if r.get("host"):
                if r["host"] == host:
                    return
                raise ConfigError(f"{robot_id} already has host {r['host']!r}")
            r["host"] = host
            path.write_text(json.dumps(robots, indent=2) + "\n")
            return
    raise ConfigError(f"no robots.json entry {robot_id!r}")


# ---- probe ------------------------------------------------------------------------------------
def parse_probe(text: str) -> dict[str, str]:
    out = {}
    for line in text.splitlines():
        k, sep, v = line.partition("=")
        if sep and re.match(r"^[a-z0-9_./]+$", k):
            out[k] = v.strip()
    return out


def _ver(s: str) -> tuple[int, ...]:
    try:
        return tuple(int(x) for x in s.split("."))
    except ValueError:
        return ()


def _num(s: str | None) -> float | None:
    try:
        return float(s) if s not in (None, "") else None
    except ValueError:
        return None


def local_shas(local_dir: Path) -> dict[str, str]:
    return {f: hashlib.sha256((local_dir / f).read_bytes()).hexdigest()[:12]
            for f in STAGED if (local_dir / f).is_file()}


def evaluate(p: dict[str, str], robot_id: str, team: str, tag: int, profile: str = "stock",
             local: dict[str, str] | None = None, other_fps: dict[str, str] | None = None) -> list[tuple[str, str, str]]:
    """(level, item, detail) rows. FAIL = `fcctl.sh start <profile>` cannot work; WARN = look at it."""
    rows: list[tuple[str, str, str]] = []

    def add(ok, item, detail, bad="FAIL"):
        rows.append(("PASS" if ok else bad, item, detail))

    if p.get("probe_done") != "yes":
        return [("FAIL", "probe", "probe did not finish (ssh output cut short?)")]
    host = p.get("hostname", "")
    add(True, "pi", f"{host} | {p.get('model', '?')} | {p.get('os', '?')} | {p.get('arch', '?')}")
    add(p.get("user") == "pi", "user", f"{p.get('user')} (profiles hard-code /home/pi)")
    # identity: a clone of another robot's card shares these
    for oid, fp in (other_fps or {}).items():
        if fp and p.get("ssh_hostkey_fp") == fp:
            add(False, "identity", f"ssh host key == robot {oid}'s: this is an uncorrected CLONE. Run "
                                   "provision.sh --fix-clone-identity <new-hostname> (see PROVISION.md)")
    if robot_id != "blue" and re.search(r"\bmts\b|mts-pi", f"{host} {p.get('tailscale_name', '')}"):
        add(False, "identity", f"hostname {host!r} / tailscale {p.get('tailscale_name')!r} look like mts: clone "
                               "identity not fixed", bad="WARN")
    if p.get("appliance_mode") == "yes":
        add(False, "appliance", "codex appliance mode markers present (/etc/fc-appliance-boot): harmless for "
                                "stock, but a sign this card came from mts", bad="WARN")
    add(True, "network", f"lan {p.get('lan_ips') or '-'} | wifi {p.get('wifi_ssid') or '-'} | tailscale "
                         f"{p.get('tailscale_ip') or '-'} ({p.get('tailscale_name') or '-'})")
    # stack prerequisites (stock profile)
    add(p.get("ros_jazzy") == "yes", "ros", f"/opt/ros: {p.get('ros_distros') or 'none'} (need jazzy)")
    add(p.get("mono") == "yes", "monorepo", f"~/pupperv3-monorepo/ros2_ws/install rev {p.get('mono_rev') or '?'}, "
                                            f"{p.get('mono_dirty_files') or '?'} locally modified files")
    missing = [k for k in STOCK_PKGS if p.get(f"pkg_{k}") in (None, "", "missing")]
    add(not missing, "packages", "all stock packages found" if not missing else f"missing: {', '.join(missing)}")
    add(p.get("stock_config") == "yes", "config.yaml", "neural_controller launch/config.yaml " +
        ("installed" if p.get("stock_config") == "yes" else "MISSING"))
    mux = p.get("launch_starts_mux")
    add(mux == "no", "launch.py mux",
        "installed launch.py starts no cmd_vel_mux (matches mts)" if mux == "no" else
        "no installed neural_controller launch.py" if p.get("launch_py") != "yes" else
        "installed launch.py STARTS its own cmd_vel_mux (newer class revision): with stock.sh's mux that is 2 "
        "/cmd_vel publishers and start will refuse READY. Needs a profile variant without stock.sh's mux")
    add(p.get("launch_teleop_arg") == "yes", "launch.py teleop", "teleop:=False honoured", bad="WARN")
    sha = p.get("policy_sha", "")
    add(sha.startswith(MTS_POLICY_SHA), "policy", f"{p.get('policy_path') or '?'} sha {sha or '?'} "
                                                  f"(mts: {MTS_POLICY_SHA}; different = different gait, re-run step tests)",
        bad="WARN")
    add(_ver(p.get("python", "")) >= (3, 10), "python", p.get("python") or "not found")
    add(p.get("rclpy") == "yes", "rclpy", "rclpy + geometry_msgs import under jazzy" if p.get("rclpy") == "yes"
        else "cannot import rclpy/geometry_msgs: the bridge will not run")
    free = _num(p.get("disk_free_gb"))
    add(free is not None and free >= MIN_DISK_GB, "disk", f"{p.get('disk_free_gb')} GB free of "
                                                         f"{p.get('disk_total_gb')} GB on {p.get('root_dev') or '?'}")
    add((_num(p.get("spidev")) or 0) > 0, "motor SPI", f"{p.get('spidev')} /dev/spidev* (motor board)")
    add((_num(p.get("i2c")) or 0) > 0, "IMU I2C", f"{p.get('i2c')} /dev/i2c-* (BNO IMU)", bad="WARN")
    add((_num(p.get("gamepad_paired")) or 0) > 0, "gamepad", "DualSense paired" if (_num(p.get("gamepad_paired"))
        or 0) > 0 else "no paired DualSense: R3 e-stop unavailable. Pair one (pair_ps5_controller.sh) before motion")
    add(True, "camera", f"{p.get('video')} /dev/video* (info only: fc uses the laptop's overhead camera)")
    t = _num(p.get("temp_c"))
    add(t is None or t < 75, "temperature", f"{p.get('temp_c') or '?'} C", bad="WARN")
    busy = {k: p.get(f"running_{k}") for k in ("control", "launch", "session", "bridge")}
    busy = {k: v for k, v in busy.items() if v not in (None, "", "0")}
    add(not busy, "idle", "no robot stack running" if not busy else
        f"already running: {busy} (fcctl start refuses; ask who owns it)", bad="WARN")
    # our files
    staged_id = p.get("fc_robot_id", "")
    add(staged_id == robot_id, "robot_id", f"~/fc-v1/robot/robot_id = {staged_id or '(none)'} (want {robot_id})")
    add(p.get("fc_link_key_mode") == "600", "link.key", f"mode {p.get('fc_link_key_mode') or '(missing)'}")
    if local:
        stale = [f for f, s in local.items() if p.get(f"fc_sha_{f}") != s]
        add(not stale, "fc files", "staged files match this laptop" if not stale else f"differ/missing: {', '.join(stale)}")
    ident = p.get("fc_identity", "")
    try:
        ij = json.loads(ident) if ident else {}
    except json.JSONDecodeError:
        ij = {}
    add(ij.get("robot_id") == robot_id and ij.get("team") == team and ij.get("tag") == tag, "identity.json",
        ident or "(none)", bad="WARN")
    if profile != "stock":
        add(False, "profile", f"robots.json profile {profile!r}: only stock prerequisites are checked here", bad="WARN")
    return rows


def render(rows) -> tuple[str, bool]:
    ready = not any(r[0] == "FAIL" for r in rows)
    w = max(len(r[1]) for r in rows)
    lines = [f"{lvl:<4}  {item:<{w}}  {detail}" for lvl, item, detail in rows]
    n = {k: sum(r[0] == k for r in rows) for k in ("PASS", "WARN", "FAIL")}
    lines.append(f"\nREADY for `fcctl.sh start`: {'yes' if ready else 'NO'}  ({n['PASS']} pass, {n['WARN']} warn, "
                 f"{n['FAIL']} fail)")
    return "\n".join(lines), ready


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["config", "set-host", "report"])
    ap.add_argument("--robots", type=Path, required=True)
    ap.add_argument("--robot-id", required=True)
    ap.add_argument("--team", default="")
    ap.add_argument("--tag", type=int, default=0)
    ap.add_argument("--host", default="")
    ap.add_argument("--probe", type=Path)
    ap.add_argument("--local-dir", type=Path)
    ap.add_argument("--known-hosts-fp", action="append", default=[], help="OTHER_ROBOT_ID=SHA256:...")
    a = ap.parse_args(argv)
    try:
        robots = json.loads(a.robots.read_text())
        if a.cmd == "config":
            print(config_action(robots, a.robot_id, a.team, a.tag, a.host))
            return 0
        if a.cmd == "set-host":
            set_host(a.robots, a.robot_id, a.host)
            print(f"robots.json: {a.robot_id}.host = {a.host}")
            return 0
    except ConfigError as e:
        print(f"config: {e}", file=sys.stderr)
        return 2
    entry = next((r for r in robots if r.get("id") == a.robot_id), {})
    fps = dict(kv.split("=", 1) for kv in a.known_hosts_fp if "=" in kv)
    rows = evaluate(parse_probe(a.probe.read_text()), a.robot_id, a.team, a.tag, entry.get("profile", "stock"),
                    local_shas(a.local_dir) if a.local_dir else None, fps)
    text, ready = render(rows)
    print(text)
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
