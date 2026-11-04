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
    missing_local = []
    if state.get("schema") != "pupper_current_session_v2":
        raise ValueError("Unknown current-state schema; inspect the handoff before proceeding")
    def git(*args):
        return subprocess.check_output(["git", "-C", str(root), *args], text=True, timeout=5).strip()
    branch, head = git("branch", "--show-current"), git("rev-parse", "--short", "HEAD")
    dirty = len(git("status", "--porcelain").splitlines())
    primary = state["branch"]
    branch_note = None
    if not branch:
        errors.append(f"Detached HEAD. Switch to a named branch to use the saved {primary} handoff.")
    elif branch != primary and branch not in state.get("legacy_branches", []):
        for ref in (f"refs/heads/{primary}", f"refs/remotes/origin/{primary}"):
            common = subprocess.run(["git", "-C", str(root), "merge-base", "HEAD", ref],
                                    capture_output=True, timeout=5)
            if common.returncode == 0:
                branch_note = f"{branch} shares history with {ref}. Showing this checkout's saved {primary} handoff."
                break
        else:
            errors.append(f"Cannot establish shared history between {branch} and {primary}. "
                          f"Check the checkout and fetch {primary} history if it is missing or shallow.")
    refs = [state["current_report"], state["historical_recovery"]["path"], *state["evidence"].values()]
    for ref in dict.fromkeys(refs):
        p = Path(ref)
        if p.is_absolute() or ".." in p.parts:
            raise ValueError(f"Invalid evidence reference: {ref}")
        if not (root / p).is_file():
            errors.append(f"Missing evidence reference: {ref}")
    py = root / state["environment"]["python"]
    if not py.is_file():
        missing_local.append(f"API test interpreter: {py}")
    hardware = Path(state["environment"]["frozen_hardware_inputs"])
    if hardware.is_absolute() or ".." in hardware.parts:
        raise ValueError(f"Invalid frozen hardware input path: {hardware}")
    if not (root / hardware).is_file():
        missing_local.append(f"Frozen hardware archive: {hardware}")
    if verify_inputs:
        errors.extend(f"Required for verification: {item}" for item in missing_local)
        for name, expected in [
            (state["historical_recovery"]["path"], state["historical_recovery"]["sha256"]),
            (state["environment"]["frozen_hardware_inputs"], state["environment"]["frozen_hardware_sha256"]),
        ]:
            p = root / name
            # Missing tracked reports and local archives are reported above.
            if p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest() != expected:
                errors.append(f"Pinned input checksum mismatch: {name}")
    return {"mode": state["mode"], "updated_utc": state["updated_utc"],
            "branch": branch, "branch_note": branch_note, "head": head, "changed_paths": dirty,
            "automation": state["activity"]["automation"],
            "robot_last_observation": state["hardware_session"]["status"],
            "robot_status_is_live": False, "test_python": str(py),
            "next": state["next"], "activity": state["activity"],
            "errors": errors, "local_references_ok": not errors,
            "missing_local": missing_local, "verify_inputs_requested": verify_inputs,
            "scope": "Local handoff checks only; no readiness for physical motion implied."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--verify-inputs", action="store_true", help="Require the local API test interpreter and verify both pinned input digests")
    args = parser.parse_args()
    try:
        r = inspect(verify_inputs=args.verify_inputs)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print(f"Handoff check failed: {exc}")
        return 1
    if args.json:
        print(json.dumps(r, indent=2))
    else:
        print(f"{r['mode']} | {r['branch'] or 'detached HEAD'} @ {r['head']} | {r['changed_paths']} changed paths")
        if r["branch_note"]:
            print(r["branch_note"])
        print(f"Automation: {r['automation']} | local references: {'OK' if r['local_references_ok'] else 'ERROR'}")
        print(f"Robot (last observation, not a live check): {r['robot_last_observation']}")
        print(f"Configured API test Python: {r['test_python']}")
        if not r["verify_inputs_requested"]:
            print("Pinned input digests: unchecked (use --verify-inputs for strict verification)")
            for item in r["missing_local"]:
                print(f"Local setup missing: {item}")
        owners = r['activity'].get('owners', {})
        if owners:
            print(f"Saved owners/activity ({r['updated_utc']}; verify actual agents and exits before resuming):")
            for owner, activity in owners.items():
                print(f"  {owner}: {activity}")
        print("Next actions:")
        for i, action in enumerate(r['next'], 1):
            print(f"  {i}. [{action['id']}] {action['action']}")
        for error in r['errors']:
            print(f"ERROR: {error}")
        print("Details: foundation/results/interruption-recovery.json")
    return 0 if not r['errors'] else 1


if __name__ == "__main__":
    raise SystemExit(main())
