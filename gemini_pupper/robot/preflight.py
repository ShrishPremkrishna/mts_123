#!/usr/bin/env python3
"""Read local configuration and artifacts; never import ROS or start a controller."""

from __future__ import annotations

import json
import sys
import contextlib
import importlib.util
import io

from runtime_config import REPO, component_dir, follower_config
from tricks import load_tricks, reference_message_data


def inspect() -> dict:
    result = {"ready": True, "checks": {}, "errors": []}
    validator_path = component_dir("pupper_gait_deploy", "PUPPER_GAIT_DEPLOY_DIR") / "download_policy.py"
    try:
        c = follower_config()
        result["checks"]["follower"] = {"mode": c.mode, "service_dir": str(c.service_dir) if c.service_dir else None,
                                           "use_color": c.use_color, "avoid_obstacles": c.avoid_obstacles,
                                           "always_detect": c.always_detect}
    except (ValueError, OSError) as exc:
        result["errors"].append(str(exc))
    try:
        tricks = load_tricks()
        result["checks"]["installed_tricks"] = sorted(n for n, t in tricks.items() if not t.secret)
        result["checks"]["hidden_trick_count"] = sum(t.secret for t in tricks.values())
        result["checks"]["training_artifacts_pending"] = [n for n in ("sit_stay_shake", "spider_walk") if n not in tricks]
        if validator_path.is_file():
            spec = importlib.util.spec_from_file_location("pupper_deploy_validator", validator_path)
            validator = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(validator)
            checked = 0
            for trick in tricks.values():
                try:
                    if trick.kind == "reference":
                        reference_message_data(trick)
                    else:
                        with contextlib.redirect_stdout(io.StringIO()):
                            validator.validate(trick.path)
                            if trick.walk_path:
                                validator.validate(trick.walk_path)
                    checked += 1
                except (ValueError, KeyError, OSError, SystemExit) as exc:
                    result["errors"].append(f"Invalid policy contract for {trick.name}: {exc}")
            result["checks"]["validated_trick_contracts"] = checked
    except (ValueError, KeyError, OSError) as exc:
        result["errors"].append(f"Invalid trick artifacts: {exc}")
    for name, path in {
        "web_app": REPO / "web/dist/index.html",
        "policy_validator": validator_path,
        "ros_workspace": component_dir("pupperv3-monorepo", "PUPPER_MONOREPO_DIR") / "ros2_ws/install/local_setup.bash",
    }.items():
        result["checks"][name] = {"path": str(path), "exists": path.is_file()}
        if not path.is_file():
            result["errors"].append(f"Missing {name}: {path}")
    result["ready"] = not result["errors"]
    result["scope"] = "Local startup files and policy data contracts only; no hardware, ROS service, API, calibration or policy performance was tested."
    return result


if __name__ == "__main__":
    report = inspect()
    print(json.dumps(report, indent=2))
    sys.exit(0 if report["ready"] else 1)
