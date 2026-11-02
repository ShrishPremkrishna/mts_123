import importlib.util
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo

import pytest

SOURCE = Path(__file__).parents[1] / "scripts" / "report_lab6_session.py"
spec = importlib.util.spec_from_file_location("session_report", SOURCE)
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def job(event, sequence, seconds, **changes):
    row = dict(schema="pupper_job_event_v1", server_instance="server", job_id="server:1",
               event=event, event_seq=sequence, wall_time_s=1000+seconds,
               monotonic_s=seconds, name="move", execution_started=event != "queued",
               outcome="succeeded" if event == "finished" else None, stop_generation=0)
    row.update(changes)
    return {"kind": "job", "job": row}


def test_explicit_timezone_and_dst_ambiguity():
    zone = ZoneInfo("America/Los_Angeles")
    assert report.naive_epoch("2026-11-01 20:18:12,613", zone) == pytest.approx(1793593092.613)
    with pytest.raises(ValueError, match="ambiguous"):
        report.naive_epoch("2026-11-01 01:30:00,000", zone)
    with pytest.raises(ValueError, match="nonexistent"):
        report.naive_epoch("2026-03-08 02:30:00,000", zone)


def test_http_access_never_borrows_adjacent_time():
    text = 'INFO: 127.0.0.1 - "POST /animation HTTP/1.1" 200 OK\n2026-11-01 20:18:12,613 robot_api INFO trick sit: completed\n'
    events, untimed, gaps = report.parse_log(text, "api.log", ZoneInfo("America/Los_Angeles"))
    assert len(events) == len(untimed) == 1 and not gaps
    assert untimed[0]["at"] is None
    assert events[0]["line"] == 2


def test_structured_job_clocks_and_cancel_before_start():
    rows = [job("queued", 1, 1), job("started", 3, 4), job("cancel_requested", 4, 5),
            job("finished", 5, 7, outcome="cancelled")]
    result = report.job_timings(rows)[0]
    assert result["queue_wait_s"] == 3 and result["execution_s"] == 3
    assert not result["issues"] and not result["cancel_before_start"]
    cancelled = report.job_timings([job("queued", 1, 1), job("finished", 2, 2, outcome="cancelled", execution_started=False)])[0]
    assert cancelled["cancel_before_start"] and cancelled["execution_s"] is None
    assert cancelled["queue_wait_s"] is None


@pytest.mark.parametrize("rows", [
    [job("started", 1, 1), job("finished", 2, 2)],
    [job("queued", 1, 1), job("started", 1, 2), job("finished", 3, 3)],
    [job("queued", 1, 5), job("started", 2, 4), job("finished", 3, 6)],
    [job("queued", 1, 1), job("started", 2, 2), job("finished", 3, 3, execution_started=False)],
])
def test_incomplete_or_inconsistent_job_does_not_invent_timings(rows):
    result = report.job_timings(rows)[0]
    assert result["issues"] and result["execution_s"] is None and result["queue_wait_s"] is None


def test_cancel_request_does_not_override_successful_terminal_result():
    result = report.job_timings([job("queued", 1, 1), job("started", 2, 2),
        job("cancel_requested", 3, 3), job("finished", 4, 4)])[0]
    assert result["events"][-1]["outcome"] == "succeeded"
    assert not result["cancel_before_start"]


def test_same_job_name_different_server_is_not_merged():
    first = job("queued", 1, 1)
    second = job("finished", 2, 2, server_instance="other", job_id="other:1")
    assert len(report.job_timings([first, second])) == 2


def test_structured_payload_timestamp_not_logger_prefix():
    payload = job("queued", 1, 1)["job"]
    events, _, gaps = report.parse_log("2026-11-01 20:18:12,613 robot_api INFO job_event " + json.dumps(payload), "api.log", ZoneInfo("UTC"))
    assert not gaps and events[0]["at"] == 1001


def test_malformed_json_shapes_are_gaps_not_crashes(tmp_path):
    events, _, gaps = report.parse_log('job_event []\njob_event null\n', 'api.log', ZoneInfo('UTC'))
    assert not events and len(gaps) == 2
    root = tmp_path/'capture'; root.mkdir(); camera = root/'camera'; camera.mkdir()
    (root/'operator-actions.jsonl').write_text('[]\nnull\n')
    (camera/'frames.jsonl').write_text('[]\nnull\n')
    result = report.build_report(root, tmp_path/'out', 'UTC', 1)
    assert sum('not an object' in g['problem'] for g in result['data_gaps']) == 4


def test_reversed_cancel_without_start_is_invalid():
    result = report.job_timings([job('queued', 2, 3), job('finished', 1, 2, outcome='cancelled', execution_started=False)])[0]
    assert result['issues'] and not result['cancel_before_start']


def test_topic_coverage_does_not_bridge_bags():
    bags = [{'topics': {report.IMU: {'start': 2., 'end': 4.}}}, {'topics': {report.IMU: {'start': 6., 'end': 8.}}}]
    assert report.topic_coverage(bags, 1., 9.)[report.IMU]['outside_recorded_extents'] == [[1., 2.], [4., 6.], [8., 9.]]


def test_overlap_is_explicit_and_topic_specific():
    a = {"source": "a", "topics": {"/imu": {"start": 1., "end": 3.}}}
    b = {"source": "b", "topics": {"/imu": {"start": 2., "end": 4.}}}
    c = {"source": "c", "topics": {"/cmd_vel": {"start": 2., "end": 4.}}}
    overlaps = report.overlap_records([a, b, c])
    assert len(overlaps) == 1 and overlaps[0]["start"] == 2 and overlaps[0]["end"] == 3


def test_camera_path_escape_rejected(tmp_path):
    root = tmp_path / "capture"; camera = root / "camera"; camera.mkdir(parents=True)
    meta = camera / "frames.jsonl"
    with pytest.raises(ValueError, match="escapes"):
        report.safe_frame_path(root, meta, "../../outside.jpg")
    with pytest.raises(ValueError, match="relative"):
        report.safe_frame_path(root, meta, "/tmp/outside.jpg")


def test_periodic_status_does_not_flood_event_timeline():
    clip = "sit_hold playing frame {} queued 0 execution run1 request none"
    assert report.meaningful_status("/trick/clip_status", clip.format(4)) == report.meaningful_status("/trick/clip_status", clip.format(50))
    assert report.meaningful_status("/trick/clip_status", clip.format(4)) != report.meaningful_status("/trick/clip_status", clip.format(4).replace("playing", "holding"))
    first = {"latched": False, "epoch": 1, "sequence": 12, "at": 123}
    later = dict(first, sequence=100, at=456)
    assert report.meaningful_status("/stop_status", json.dumps(first)) == report.meaningful_status("/stop_status", json.dumps(later))
    later["latched"] = True
    assert report.meaningful_status("/stop_status", json.dumps(first)) != report.meaningful_status("/stop_status", json.dumps(later))


def test_request_aggregation_drops_secrets_and_free_text():
    source = {"path": "/tools/unlock", "name": "my personal phrase", "body": {"password": "DO_NOT_COPY", "name": "secret"}, "response": {"message": "DO_NOT_COPY", "success": True}}
    assert "DO_NOT_COPY" not in json.dumps(report.action_record(source))
    assert report.action_record(source)["body"] == {}
    source = {"path": "/animation", "body": {"animation_name": "sit_stay_shake", "user_text": "DO_NOT_COPY"}}
    assert report.action_record(source)["body"] == {"animation_name": "sit_stay_shake"}


def write_twist(path, times):
    from mcap_ros2.writer import Writer
    definition = "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n" + "="*80 + "\nMSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n"
    with path.open("wb") as stream:
        writer = Writer(stream)
        schema = writer.register_msgdef("geometry_msgs/msg/Twist", definition)
        for index, time in enumerate(times):
            writer.write_message("/cmd_vel", schema, {"linear": {"x": .2 if index == 0 else 0., "y": 0., "z": 0.}, "angular": {"x": 0., "y": 0., "z": 0.}}, log_time=int(time*1e9), publish_time=int(time*1e9))
        writer.finish()


def test_actual_mcap_decoder_and_partial_tail(tmp_path):
    path = tmp_path / "a.mcap"; write_twist(path, [1001., 1002.])
    info, streams, events = report.read_bag(path, "a.mcap")
    assert not info["error"] and info["topics"]["/cmd_vel"]["nonzero_messages"] == 1
    assert streams["/cmd_vel"].shape == (2, 4) and not events
    broken = tmp_path / "broken.mcap"; broken.write_bytes(path.read_bytes()[:-60])
    info, _, _ = report.read_bag(broken, "broken.mcap")
    assert info["error"]  # report may preserve partial data, never call it complete


def test_complete_report_is_self_contained_and_inputs_unchanged(tmp_path):
    from PIL import Image
    root = tmp_path / "capture"; root.mkdir()
    write_twist(root / "one.mcap", [1001., 1002.])
    write_twist(root / "overlap.mcap", [1001.5, 1002.5])
    shutil.copyfile(root / "one.mcap", root / "duplicate.mcap")
    (root / "api.log").write_text('INFO: "POST /stop/immediate HTTP/1.1" 200 OK\n')
    (root / "launch.log").write_text('[INFO] [1000000001.0] [test]: activated\n')
    camera = root / "camera"; camera.mkdir(); frame = camera / "frame.jpg"
    Image.new("RGB", (32, 24), "blue").save(frame)
    (camera / "frames.jsonl").write_text(json.dumps({"arrival_wall_s": 1001.5, "file": "frame.jpg", "sha256": report.sha(frame)}) + "\n")
    output = tmp_path / "out"
    result = report.build_report(root, output, "UTC", 1)
    assert result["input_unchanged_after_read"] and len(result["bags"]) == 2
    assert len(result["bag_overlaps"]) == 1 and len(result["camera_thumbnails"]) == 1
    assert any("duplicate bag" in gap["problem"] for gap in result["data_gaps"])
    html = (output / "report.html").read_text()
    assert "data:image/png;base64," in html and "data:image/jpeg;base64," in html
    assert "src='http" not in html and result["untimed_http_access"][0]["at"] is None
    with pytest.raises(FileExistsError): report.build_report(root, output, "UTC")
    with pytest.raises(ValueError): report.build_report(root, root / "nested-output", "UTC")


def test_bad_camera_hash_is_not_displayed(tmp_path):
    from PIL import Image
    root = tmp_path / "capture"; root.mkdir(); camera = root / "camera"; camera.mkdir()
    Image.new("RGB", (4, 4)).save(camera / "frame.jpg")
    (camera / "frames.jsonl").write_text(json.dumps({"arrival_wall_s": 1001., "file": "frame.jpg", "sha256": "0"*64}) + "\n")
    result = report.build_report(root, tmp_path / "out", "UTC", 1)
    assert not result["camera_thumbnails"]
    assert any("hash mismatch" in gap["problem"] for gap in result["data_gaps"])
