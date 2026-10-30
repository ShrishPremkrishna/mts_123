#!/usr/bin/env python3
"""Print current handoff and local readiness. Never contacts or starts the robot."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT / "foundation/results/interruption-recovery.json"


def inspect(root=ROOT, verify_inputs=False):
    state = json.loads((root / STATE.relative_to(ROOT)).read_text())
    errors = []
    if state.get("schema") != "pupper_current_session_v2":
        raise ValueError("Unknown current-state schema; inspect the handoff before proceeding")
    def git(*args):
        return subprocess.check_output(["git", "-C", str(root), *args], text=True, timeout=5).strip()
    branch, head = git("branch", "--show-current"), git("rev-parse", "--short", "HEAD")
    dirty = len(git("status", "--porcelain").splitlines())
    if branch != state["branch"]:
        errors.append(f"Expected branch {state['branch']}, found {branch}")
    refs = [state["current_report"], state["historical_recovery"]["path"], *state["evidence"].values()]
    for ref in refs:
        p = Path(ref)
        if p.is_absolute() or ".." in p.parts or not (root / p).is_file():
            errors.append(f"Missing or invalid evidence reference: {ref}")
    py = root / state["environment"]["python"]
    if not py.is_file():
        errors.append(f"Test interpreter unavailable: {py}")
    if verify_inputs:
        for name, expected in [
            (state["historical_recovery"]["path"], state["historical_recovery"]["sha256"]),
            (state["environment"]["frozen_hardware_inputs"], state["environment"]["frozen_hardware_sha256"]),
        ]:
            p = root / name
            if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
                errors.append(f"Pinned local input missing or changed: {name}")
    return {"mode": state["mode"], "updated_utc": state["updated_utc"],
            "branch": branch, "head": head, "changed_paths": dirty,
            "automation": state["activity"]["automation"],
            "robot_last_observation": state["hardware_session"]["status"],
            "robot_status_is_live": False, "test_python": str(py),
            "next": state["next"], "activity": state["activity"],
            "errors": errors, "local_references_ok": not errors,
            "scope": "Local handoff checks only; no readiness for physical motion implied."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--verify-inputs", action="store_true", help="Also verify the small pinned history/input files")
    args = parser.parse_args()
    try:
        r = inspect(verify_inputs=args.verify_inputs)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print(f"Handoff check failed: {exc}")
        return 1
    if args.json:
        print(json.dumps(r, indent=2))
    else:
        print(f"{r['mode']} | {r['branch']} @ {r['head']} | {r['changed_paths']} changed paths")
        print(f"Automation: {r['automation']} | local references: {'OK' if r['local_references_ok'] else 'ERROR'}")
        print(f"Robot (last observation, not a live check): {r['robot_last_observation']}")
        print(f"Test Python: {r['test_python']}")
        print("Next actions:")
        for i, action in enumerate(r['next'], 1):
            print(f"  {i}. [{action['id']}] {action['action']}")
        for error in r['errors']:
            print(f"ERROR: {error}")
        print("Details: foundation/results/interruption-recovery.json")
    return 0 if not r['errors'] else 1


if __name__ == "__main__":
    raise SystemExit(main())
