#!/usr/bin/env python3
"""Heading-hold-off overlays for the walker, no rebuild (lane D, 2026-11-23).

Why: neural_controller's heading hold (command_filter.hpp HeadingHold) replaces
the yaw command with kp * (captured_yaw - imu_yaw), clipped, whenever the robot
walks (|v| > 0.05) with |wz command| < 0.1. With the drifting BNO yaw that turns
the robot for real, and it overrides small laptop yaw commands. The camera loop
on the laptop closes yaw instead, so the hold should be off.

Where it lives (verified 2026-11-23):
  - u6 gemini_walk: static-idle-native-20261102 neural_controller.cpp:291-319
    (md5 822e068f...), policy default_policy.json (sha ecf3a4ab...) carries
    "heading_hold": {"kp": 1.5, "clip": 1.0, ...}; kp <= 0 disables it.
  - stock monorepo neural_controller (Pi working tree, rebuilt 2026-10-12):
    same logic (neural_controller.cpp:264-275); launch/default_policy.json has the
    same block. legacy_walk_policy.json has no gait_reference, so no hold.
The hold is configured only from the policy JSON, so the overlay is a policy
copy with heading_hold.kp = 0 plus a way to point the walker at it.

Subcommands (run on the Pi with system python3; writes only under OUT_DIR,
which must lie inside --root, default $FC_ROOT or ~/fc-v1):
  policy SRC OUT     byte-minimal copy of SRC with heading_hold.kp = 0
  u6 OUT_DIR         copy of the pinned u6 launch dir whose generated controller
                     config points neural_controller_default and gemini_walk at
                     the kp=0 policy (sha-pinned inside the launch file)
  stock OUT_DIR      launch_hh0.py + config.yaml for `ros2 launch`, stock line
u6/stock re-read what they wrote and verify it (only kp and the policy path
differ from the sources) before printing one JSON line; any failure exits non-zero.

How a profile applies it: profiles/stock_hh0.sh and tej_u6_hh0.sh set
FC_HEADING_HOLD=0 and source the base profile, whose profile_start calls
fcctl.sh now builds every walker overlay with ../stack_overlay.py (which reuses these helpers); this
CLI remains for inspection. See fc/robot/README.md.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys

HOME = os.path.expanduser("~")
U6_HW = HOME + "/pupper-foundation-session-20261029/software-ballkick-20261120-u6/foundation/hardware"
U6_POLICY = HOME + "/pupper-foundation-session-20261029/static-idle-native-20261102/install/share/neural_controller/launch/default_policy.json"
U6_POLICY_SHA = "ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712"
U6_LAUNCH_ANCHOR = "    controllers = runtime_config.session_skill_controllers(selected)\n"
STOCK_NC = HOME + "/pupperv3-monorepo/ros2_ws/src/neural_controller/launch"
# What `ros2 launch neural_controller launch.py` (profile stock) really runs; the overlay is built from
# STOCK_NC, so both must be byte-identical or the A/B would compare different launches.
STOCK_INSTALLED = HOME + "/pupperv3-monorepo/ros2_ws/install/neural_controller/share/neural_controller/launch"
STOCK_FILES = ("launch.py", "config.yaml", "default_policy.json")
STOCK_NEEDLE = '"$(find-pkg-share neural_controller)/launch/default_policy.json"'
STOCK_LAUNCH_PAT = re.compile(
    r'PathJoinSubstitution\(\s*\[FindPackageShare\("neural_controller"\), "launch", "config\.yaml"\]\s*\)')


class OverlayError(SystemExit):
    """Refusal: the overlay could not be built or did not verify (exit status 1 with the message)."""


def check_out_dir(out_dir, root):
    """OUT_DIR must resolve inside ROOT (symlinks resolved), so nothing is written elsewhere."""
    out, base = os.path.realpath(out_dir), os.path.realpath(root)
    if os.path.commonpath([out, base]) != base or out == base:
        raise OverlayError("refusing to write %s: not inside %s" % (out, base))
    return out


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def policy_hh0(src, out, kp=0.0):
    """Copy a policy JSON with heading_hold.kp = kp, editing only that block's text."""
    text = open(src).read()
    orig = json.loads(text)
    m = re.search(r'"heading_hold"\s*:\s*\{[^{}]*\}', text)
    if m:
        block = dict(orig["heading_hold"])
        block["kp"] = kp
        new = text[:m.start()] + '"heading_hold": ' + json.dumps(block) + text[m.end():]
    elif "gait_reference" in orig:
        # No block means the controller defaults to kp 1.0 for gait policies: add one.
        block = {"kp": kp, "clip": 0.3, "yaw_threshold": 0.1, "walk_threshold": 0.05}
        end = text.rstrip().rfind("}")
        new = text[:end].rstrip() + ', "heading_hold": ' + json.dumps(block) + text[end:]
    else:
        raise OverlayError("%s has no gait_reference: heading hold is already off" % src)
    mod = json.loads(new)
    a, b = dict(orig), dict(mod)
    a.pop("heading_hold", None)
    b.pop("heading_hold", None)
    if a != b or mod["heading_hold"]["kp"] != kp:
        raise OverlayError("edit changed more than heading_hold.kp; refusing")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        f.write(new)
    return {"policy": os.path.abspath(out), "sha256": sha256(out), "source": src,
            "source_sha256": sha256(src), "heading_hold": mod["heading_hold"]}


def _u6_patch(policy, sha):
    return (
        "    # FC v1 heading-hold overlay (fc/robot/imu/heading_hold.py): the same policy\n"
        "    # with heading_hold.kp = 0, pinned by sha256. Nothing else differs.\n"
        "    fc_policy = Path(%r)\n"
        "    if hashlib.sha256(fc_policy.read_bytes()).hexdigest() != %r:\n"
        "        raise ValueError(\"FC heading-hold-off policy changed: \" + str(fc_policy))\n"
        "    models = [fc_policy.resolve() for _ in models]\n" % (policy, sha))


def _u6_launch_src():
    src = open(os.path.join(U6_HW, "lab6_session.launch.py")).read()
    if src.count(U6_LAUNCH_ANCHOR) != 1:
        raise OverlayError("u6 launch file layout changed; refusing to patch")
    return src


def build_u6(out_dir, policy=None):
    """Overlay launch dir for persistent_session.py prepare --launch/--config/--urdf."""
    out_dir = os.path.abspath(out_dir)
    policy = policy or os.path.join(out_dir, "default_policy-hh0.json")
    if sha256(U6_POLICY) != U6_POLICY_SHA:
        raise OverlayError("u6 walking policy is not the demonstrated one; refusing")
    pol = policy_hh0(U6_POLICY, policy)
    hw = os.path.join(out_dir, "foundation", "hardware")
    os.makedirs(hw, exist_ok=True)
    src = _u6_launch_src()
    patched = src.replace(U6_LAUNCH_ANCHOR, _u6_patch(pol["policy"], pol["sha256"]) + U6_LAUNCH_ANCHOR)
    with open(os.path.join(hw, "lab6_session.launch.py"), "w") as f:
        f.write(patched)
    shutil.copy2(os.path.join(U6_HW, "lab6-session.yaml"), hw)
    fs = os.path.join(hw, "floor_stock")
    os.makedirs(fs, exist_ok=True)
    for name in ("baseline-expanded.urdf", "shutdown_coordinator.py"):
        shutil.copy2(os.path.join(U6_HW, "floor_stock", name), fs)
    return {"launch": os.path.join(hw, "lab6_session.launch.py"),
            "config": os.path.join(hw, "lab6-session.yaml"),
            "urdf": os.path.join(fs, "baseline-expanded.urdf"), **pol}


def build_stock(out_dir, policy=None):
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    for name in STOCK_FILES:
        if sha256(os.path.join(STOCK_NC, name)) != sha256(os.path.join(STOCK_INSTALLED, name)):
            raise OverlayError("stock %s: src and installed copies differ; refusing" % name)
    policy = policy or os.path.join(out_dir, "default_policy-hh0.json")
    pol = policy_hh0(os.path.join(STOCK_NC, "default_policy.json"), policy)
    cfg = open(os.path.join(STOCK_NC, "config.yaml")).read()
    n = cfg.count(STOCK_NEEDLE)
    if not n:
        raise OverlayError("stock config.yaml has no default_policy.json model_path")
    with open(os.path.join(out_dir, "config.yaml"), "w") as f:
        f.write(cfg.replace(STOCK_NEEDLE, json.dumps(pol["policy"])))
    launch = open(os.path.join(STOCK_NC, "launch.py")).read()
    if len(STOCK_LAUNCH_PAT.findall(launch)) != 1:
        raise OverlayError("stock launch.py layout changed; refusing to patch")
    launch = STOCK_LAUNCH_PAT.sub(json.dumps(os.path.join(out_dir, "config.yaml")), launch)
    with open(os.path.join(out_dir, "launch_hh0.py"), "w") as f:
        f.write(launch)
    return {"launch": os.path.join(out_dir, "launch_hh0.py"), "config": os.path.join(out_dir, "config.yaml"),
            "model_paths_replaced": n, **pol}


def _verify_policy(r, src):
    """Re-read the written policy: pinned sha, kp == 0, every other key equal to the source."""
    if sha256(r["policy"]) != r["sha256"]:
        raise OverlayError("kp=0 policy changed after writing: " + r["policy"])
    a, b = json.load(open(src)), json.load(open(r["policy"]))
    if b.get("heading_hold", {}).get("kp") != 0:
        raise OverlayError("written policy does not have heading_hold.kp = 0")
    a.pop("heading_hold", None)
    b.pop("heading_hold", None)
    if a != b:
        raise OverlayError("written policy differs from its source beyond heading_hold")


def _same(path, expected, what):
    if open(path).read() != expected:
        raise OverlayError("%s differs from what the overlay should be: %s" % (what, path))


def verify_u6(r):
    """Check the u6 overlay on disk: only the policy (kp) and the inserted, sha-pinned patch differ."""
    _verify_policy(r, U6_POLICY)
    src = _u6_launch_src()
    _same(r["launch"], src.replace(U6_LAUNCH_ANCHOR, _u6_patch(r["policy"], r["sha256"]) + U6_LAUNCH_ANCHOR),
          "u6 overlay launch")
    hw = os.path.dirname(r["launch"])
    for rel in ("lab6-session.yaml", "floor_stock/baseline-expanded.urdf", "floor_stock/shutdown_coordinator.py"):
        if sha256(os.path.join(hw, rel)) != sha256(os.path.join(U6_HW, rel)):
            raise OverlayError("u6 overlay copy differs from the release: " + rel)
    if os.path.dirname(r["config"]) != hw or os.path.dirname(os.path.dirname(r["urdf"])) != hw:
        raise OverlayError("u6 overlay config/urdf are not launch siblings (prepare would refuse)")
    return True


def verify_stock(r):
    """Check the stock overlay on disk: config differs only in the default_policy model paths."""
    _verify_policy(r, os.path.join(STOCK_NC, "default_policy.json"))
    cfg = open(os.path.join(STOCK_NC, "config.yaml")).read()
    if cfg.count(STOCK_NEEDLE) != r["model_paths_replaced"]:
        raise OverlayError("stock config.yaml changed since the overlay was built")
    _same(r["config"], cfg.replace(STOCK_NEEDLE, json.dumps(r["policy"])), "stock overlay config")
    launch = open(os.path.join(STOCK_NC, "launch.py")).read()
    _same(r["launch"], STOCK_LAUNCH_PAT.sub(json.dumps(r["config"]), launch), "stock overlay launch")
    compile(open(r["launch"]).read(), r["launch"], "exec")
    return True


def build(kind, out_dir, root, policy=None):
    """Build + verify one overlay; every written path must lie inside root."""
    out_dir = check_out_dir(out_dir, root)
    if policy:
        check_out_dir(policy, root)
    r = build_u6(out_dir, policy) if kind == "u6" else build_stock(out_dir, policy)
    (verify_u6 if kind == "u6" else verify_stock)(r)
    return {"kind": kind, "verified": True, **r}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("policy")
    p.add_argument("src")
    p.add_argument("out")
    p.add_argument("--kp", type=float, default=0.0)
    for name in ("u6", "stock"):
        q = sub.add_parser(name)
        q.add_argument("out_dir")
        q.add_argument("--policy", help="where to write the kp=0 policy (default OUT_DIR/default_policy-hh0.json)")
        q.add_argument("--root", default=os.environ.get("FC_ROOT") or HOME + "/fc-v1",
                       help="refuse to write outside this directory (default $FC_ROOT or ~/fc-v1)")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "policy":
            r = policy_hh0(a.src, a.out, a.kp)
        else:
            r = build(a.cmd, a.out_dir, a.root, a.policy)
    except OverlayError as e:
        print("heading_hold: " + str(e.code), file=sys.stderr)
        return 1
    except (OSError, ValueError, KeyError) as e:   # missing source, bad JSON: fail loudly, never half-silent
        print("heading_hold: %s: %s" % (type(e).__name__, e), file=sys.stderr)
        return 1
    print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
