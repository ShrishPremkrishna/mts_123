"""The fieldcam-to-kick path runs without a camera, ROS, or motor services."""
import pytest

from final_project.pupper_fc.kick_rehearsal import run_case


@pytest.mark.parametrize("side", ["left", "right"])
def test_walk_slows_then_commits_once_for_the_selected_paw(side):
    row = run_case("offset", side, .30)
    assert row["passed"], row
    assert row["requests"] == 1 and row["motion_commands_during_kick"] == 0
    assert row["median_command_within_10cm_m_s"] < row["peak_command_m_s"] * .6
    assert row["capture_age_p95_ms"] >= 60.


def test_silent_overhead_holds_then_resumes_on_measured_frames():
    row = run_case("offset", "right", .30, fault="camera_silence", attack_sign=-1)
    assert row["passed"], row
    assert row["moving_commands_during_silent_hold"] == 0
    assert row["camera_hold_s"] < row["resumed_steering_s"] < row["time_to_commit_s"]


def test_completed_kick_releases_exclusive_ownership_to_new_field_pose():
    row = run_case("center", "left", .30, walk_after=True)
    assert row["passed"], row
    assert row["terminal_phase"] == "target_reached"
    assert row["post_kick_move"]["displacement_m"] > .15


def test_capture_cli_selects_one_robot_and_only_admitted_events_drive_mock_kick(tmp_path, capsys):
    import json
    import math
    from final_project.pupper_fc.integration import main

    session = {"epoch": "recorded-1", "clock": "normalized-capture"}
    events = [{"event": "arm", "now": 0, **session}]
    invalid = []
    for i in range(115):
        now = i * .04
        if i % 5 == 0:
            events.append({"event": "intent", "now": now, "seq": i, "robot_id": "pupper-2",
                           "kind": "attack", "intent_id": "shot", "issued_at": now,
                           "valid_until": now + .40, **session})
        frame = {"event": "frame", "now": now, "captured_at": now, "seq": i, **session,
                 "robots": {"pupper-2": {"x": .258, "y": .040, "yaw": math.pi}},
                 "ball": {"x": 0., "y": 0.}, "goals": {"left": {"x": -1.83, "y": 0.}}}
        events.append(frame)
        if i == 25:
            invalid.extend(range(len(events), len(events) + 4))
            events.extend([dict(frame), {**frame, "epoch": "older", "now": 40.},
                           {"event": "stop", "epoch": "older", "now": 40.},
                           {"event": "arm", "now": .2, **session, "epoch": "rollback"}])
    path = tmp_path / "captured.jsonl"
    path.write_text("".join(json.dumps(event) + "\n" for event in events))
    assert main(["--replay", str(path), "--kick-side", "left", "--robot-id", "pupper-2"]) == 0
    rows = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert len(rows) == len(events)
    for i in invalid:
        assert not rows[i]["accepted"] and rows[i]["decisions"] == {}
        assert rows[i]["kick_shadow"]["simulated_receipts"] == []
        assert not rows[i]["kick_shadow"].get("request")
    assert all(set(row["decisions"]) == {"pupper-2"} for row in rows if row["accepted"])
    requests = [row["kick_shadow"]["request"] for row in rows if row["kick_shadow"].get("request")]
    assert len(requests) == 1 and requests[0]["side"] == "left"
    assert [receipt["phase"] for row in rows for receipt in row["kick_shadow"]["simulated_receipts"]] == [
        "clip_confirmed", "walker_restored"]
    assert rows[-1]["decisions"]["pupper-2"]["phase"] == "kick_complete"
