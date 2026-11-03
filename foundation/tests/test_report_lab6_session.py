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


REQUEST_A, REQUEST_B = "a" * 32, "b" * 32
EXECUTION_A, EXECUTION_B = "1" * 32, "2" * 32
CONTROLLER = "/trick_sit_stay_shake"


def clip_request(at, request=REQUEST_A, clip="shake_left", controller=CONTROLLER):
    return at, controller + "/clip", f"pupper_clip_v1 {request} {clip}"


def clip_status(at, request=REQUEST_A, execution=EXECUTION_A, clip="shake_left",
                phase="playing", frame=20, queued=0, controller=CONTROLLER):
    return at, controller + "/clip_status", f"{clip} {phase} frame {frame} queued {queued} execution {execution} request {request}"


def clip_rows(*messages):
    return [report.clip_record(topic, payload, round(at * 1e9), index)
            for index, (at, topic, payload) in enumerate(messages)]


def test_clip_early_apply_without_queued_status_keeps_repeated_receipts():
    records = clip_rows(clip_request(1), clip_status(1.02), clip_status(1.04, frame=21),
                        clip_status(2, clip="sit_hold", frame=55),
                        clip_status(3, clip="sit_hold", phase="holding", frame=55))
    result = report.clip_admission_timings(records)
    row = result["requests"][0]
    assert result["available"] and not row["issues"]
    assert row["execution_id"] == EXECUTION_A
    assert row["request_to_first_observed_applied_s"] == pytest.approx(.02)
    assert row["first_observed_status_record_index"] == 1
    assert row["matching_status_count"] == 4 and len(records) == 5


def test_clip_selected_request_id_not_queue_depth_proves_applied_status():
    result = report.clip_admission_timings(clip_rows(clip_request(1), clip_status(1.1, queued=1)))
    assert result["requests"][0]["request_to_first_observed_applied_s"] == pytest.approx(.1)


@pytest.mark.parametrize("messages, issue", [
    ([clip_status(1.1)], "request missing"),
    ([clip_request(1)], "matching applied status missing"),
    ([clip_request(1), clip_status(1.1, request=REQUEST_B)], "missing"),
    ([clip_request(1), clip_request(1.01), clip_status(1.1)], "multiple request records"),
    ([clip_request(1), clip_status(1.1), clip_status(1.2, execution=EXECUTION_B)], "multiple executions"),
    ([clip_status(.9), clip_request(1), clip_status(1.1)], "precedes request"),
    ([clip_status(1), clip_request(1)], "precedes request"),
    ([clip_request(1), clip_status(1.1, phase="holding")], "endpoint"),
    ([clip_request(1), clip_status(1.1, clip="sit_hold"), clip_status(1.2)], "endpoint"),
    ([clip_request(1), clip_status(1.1, execution="none")], "execution ID missing"),
    ([clip_request(1), clip_status(1.1, phase="stopped")], "endpoint"),
])
def test_clip_missing_ambiguous_or_retained_evidence_has_no_latency(messages, issue):
    result = report.clip_admission_timings(clip_rows(*messages))
    assert not result["available"]
    assert all(row["request_to_first_observed_applied_s"] is None for row in result["requests"])
    assert issue in json.dumps(result["requests"])


def test_clip_reappearing_selected_id_is_not_silently_deduplicated():
    rows = clip_rows(clip_request(1), clip_status(1.1), clip_request(2, REQUEST_B),
                     clip_status(2.1, request=REQUEST_B), clip_status(3))
    result = {r["request_id"]: r for r in report.clip_admission_timings(rows)["requests"]}
    assert result[REQUEST_A]["request_to_first_observed_applied_s"] is None
    assert "reappears" in result[REQUEST_A]["issues"][0]
    assert result[REQUEST_B]["request_to_first_observed_applied_s"] == pytest.approx(.1)


@pytest.mark.parametrize("payload", ["shake_left", "pupper_clip_v1 bad shake_left",
                                      f"pupper_clip_v1 {REQUEST_A} shake_left extra",
                                      f" pupper_clip_v1 {REQUEST_A} shake_left"])
def test_clip_legacy_or_malformed_requests_are_retained_without_invented_ids(payload):
    records = clip_rows((1, CONTROLLER + "/clip", payload), clip_status(1.1))
    assert records[0]["raw"] == payload and "request_id" not in records[0]
    result = report.clip_admission_timings(records)
    assert not result["available"] and result["unidentified_record_indices"] == [0]


def test_clip_old_status_only_bag_is_explicitly_unavailable():
    old = "shake_left holding frame 50 queued 0 execution " + EXECUTION_A
    records = clip_rows((1, CONTROLLER + "/clip_status", old), clip_status(2, request="none"))
    result = report.clip_admission_timings(records)
    assert not result["available"] and not result["requests"]
    assert result["unidentified_record_indices"] == [0, 1]
    assert "no versioned clip requests" in result["issues"][0]


def test_clip_status_only_bag_with_explicit_ids_keeps_missing_request_reason():
    records = clip_rows(clip_status(1, request="none"), clip_status(2), clip_status(3, request=REQUEST_B))
    result = report.clip_admission_timings(records)
    assert not result["available"] and "no versioned clip requests" in result["issues"][0]
    assert len(result["requests"]) == 2
    assert all(row["request_record_indices"] == [] and "request missing" in row["issues"]
               and row["request_to_first_observed_applied_s"] is None for row in result["requests"])


def write_clip_messages(path, messages):
    from mcap_ros2.writer import Writer
    with path.open("wb") as stream:
        writer = Writer(stream)
        schema = writer.register_msgdef("std_msgs/msg/String", "string data\n")
        for at, topic, payload in messages:
            writer.write_message(topic, schema, {"data": payload}, log_time=round(at * 1e9), publish_time=round(at * 1e9))
        writer.finish()


def test_clip_mcap_retains_full_messages_while_timeline_still_collapses(tmp_path):
    path = tmp_path / "clips.mcap"
    write_clip_messages(path, [clip_request(1), clip_status(1.02), clip_status(1.04, frame=21),
                               clip_status(1.06, frame=22)])
    info, _, events = report.read_bag(path, "clips.mcap")
    assert not info["error"] and len(info["clip_records"]) == 4
    assert len(events) == info["status_transitions"][CONTROLLER + "/clip_status"] == 1
    assert info["topics"][CONTROLLER + "/clip"]["messages"] == 1
    assert info["clip_records"][1]["log_time_ns"] == 1020000000
    assert info["clip_admission"]["requests"][0]["request_to_first_observed_applied_s"] == pytest.approx(.02)


def test_clip_multiple_executions_and_controllers_remain_distinct():
    rows = clip_rows(clip_request(1), clip_status(1.1),
                     clip_request(2, REQUEST_B), clip_status(2.2, request=REQUEST_B, execution=EXECUTION_B),
                     clip_request(3, controller="/trick_other"),
                     clip_status(3.3, execution=EXECUTION_B, controller="/trick_other"))
    result = report.clip_admission_timings(rows)
    assert all(not row["issues"] for row in result["requests"])
    assert sorted(row["request_to_first_observed_applied_s"] for row in result["requests"]) == pytest.approx([.1, .2, .3])


def test_clip_report_never_matches_across_bags_and_preserves_html_evidence(tmp_path):
    root = tmp_path / "capture"; root.mkdir()
    write_clip_messages(root / "requests-only.mcap", [clip_request(1)])
    write_clip_messages(root / "statuses-only.mcap", [clip_status(1.1)])
    write_clip_messages(root / "complete.mcap", [clip_request(1), clip_status(1.2, execution=EXECUTION_B)])
    output = tmp_path / "out"
    result = report.build_report(root, output, "UTC", 0)
    bags = {bag["source"]: bag for bag in result["bags"]}
    assert not bags["requests-only.mcap"]["clip_admission"]["available"]
    assert not bags["statuses-only.mcap"]["clip_admission"]["available"]
    row = bags["complete.mcap"]["clip_admission"]["requests"][0]
    assert row["execution_id"] == EXECUTION_B and row["request_to_first_observed_applied_s"] == pytest.approx(.2)
    assert "not physical latency" in (output / "report.html").read_text()
    assert result["input_unchanged_after_read"]


def write_summaryless_clip_bag(path, undecodable_topic):
    """Real MCAP with an unsupported channel; selected messages still use ROS CDR."""
    from mcap.reader import make_reader
    from mcap.writer import Writer as RawWriter, IndexType
    from mcap_ros2.writer import Writer
    source = path.with_suffix(".source.mcap")
    with source.open("wb") as stream:
        writer = Writer(stream)
        string = writer.register_msgdef("std_msgs/msg/String", "string data\n")
        for at, topic, payload in [clip_request(1), clip_status(1.1)]:
            writer.write_message(topic, string, {"data": payload}, log_time=round(at * 1e9), publish_time=round(at * 1e9))
        imu = writer.register_msgdef("sensor_msgs/msg/Imu", """std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
""")
        writer.write_message(report.IMU, imu, {"header": {"stamp": {"sec": 1, "nanosec": 200000000}, "frame_id": "base_link"},
            "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}, "orientation_covariance": [0.] * 9,
            "angular_velocity": {"x": 0., "y": 0., "z": .1}, "angular_velocity_covariance": [0.] * 9,
            "linear_acceleration": {"x": 0., "y": 0., "z": 9.81}, "linear_acceleration_covariance": [0.] * 9},
            log_time=1200000000, publish_time=1200000000)
        writer.finish()
    with path.open("wb") as output, source.open("rb") as stream:
        writer = RawWriter(output, index_types=IndexType.NONE, repeat_channels=False, repeat_schemas=False,
                           use_statistics=False, use_summary_offsets=False)
        writer.start()
        bad_schema = writer.register_schema("unrelated/Unknown", "unsupported", b"unknown")
        bad_channel = writer.register_channel(undecodable_topic, "unsupported", bad_schema)
        writer.add_message(bad_channel, 500000000, b"not decodable", 500000000)
        schemas, channels = {}, {}
        for schema, channel, message in make_reader(stream).iter_messages():
            if schema.id not in schemas:
                schemas[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
            if channel.id not in channels:
                channels[channel.id] = writer.register_channel(channel.topic, channel.message_encoding, schemas[schema.id])
            writer.add_message(channels[channel.id], message.log_time, message.data, message.publish_time)
        writer.finish()


def test_summaryless_bag_filters_raw_topics_before_decoding(tmp_path):
    path = tmp_path / "summaryless.mcap"
    write_summaryless_clip_bag(path, "/unrelated/unsupported")
    info, streams, _ = report.read_bag(path, path.name)
    assert info["has_summary"] is False and info["error"] is None
    assert "/unrelated/unsupported" not in info["topics"]
    assert streams[report.IMU].shape == (1, 9)
    assert info["clip_admission"]["requests"][0]["request_to_first_observed_applied_s"] == pytest.approx(.1)


def test_summaryless_selected_unsupported_channel_remains_an_explicit_error(tmp_path):
    path = tmp_path / "malformed-selected.mcap"
    write_summaryless_clip_bag(path, "/trick_other/clip_status")
    info, _, _ = report.read_bag(path, path.name)
    assert info["has_summary"] is False
    assert "DecoderNotFoundError" in info["error"]
    assert not info["clip_admission"]["available"]
