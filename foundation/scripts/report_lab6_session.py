#!/usr/bin/env python3
"""Read a closed Lab 6 recording; write a new, self-contained evidence report.

Uses the existing numerical .venv (mcap, mcap_ros2, numpy, matplotlib, Pillow).
Never invokes ROS, HTTP, subprocesses or hardware. All inputs are opened read-only.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import html
import importlib.metadata
import io
import json
import math
from pathlib import Path
import re
import sys
from zoneinfo import ZoneInfo

IMU = "/imu_sensor_broadcaster/imu"
JOINTS = "/joint_states"
COMMANDS = {"/cmd_vel", "/llm_cmd_vel"}
TOPICS = {IMU, JOINTS, *COMMANDS, "/estop_controller/stop_status", "/emergency_stop"}
CLIP_TOPIC = re.compile(r"/trick_[^/]+/(clip|clip_status)")
CLIP_ID = re.compile(r"[0-9a-f]{32}")
ANSI = re.compile(r"\x1b\[[0-9;]*m")
NATIVE = re.compile(r"\[(\d{9,}\.\d+)\]\s*\[([^]]+)\]:\s*(.*)")
API = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d[,.]\d+)\s+(\S+)\s+(\S+)\s+(.*)")
EVENT_WORDS = re.compile(r"activat|deactivat|emergency|fall|bad imu|stale imu|invalid imu|shutdown|clip.*(?:start|finish|complete)|playback.*(?:start|finish|complete)", re.I)
LIMITS = [
    "Offline evidence only: no physical-success, stability, acoustic-latency or calibration verdict.",
    "The common axis uses recorded host wall/MCAP receipt time. Cross-host clock synchronization is not established.",
    "IMU orientation and rates are reported by the same sensor. Header time is retained but is not verified sensor measurement time.",
    "Zero velocity commands do not prove physical stillness or absence of policy joint targets.",
    "Each bag is plotted separately; overlapping recordings are retained, not summed, deduplicated by timestamp, or silently concatenated.",
    "Historical name/time matches are candidate correlations without request IDs, not causal links or execution-latency measurements.",
    "Structured started/finished events describe API job invocation/result, not physical movement onset/completion.",
    "Clip admission measures same-bag request receipt to first observed applied-status receipt, not API dispatch, controller processing time or physical onset. Missing intermediate statuses can lengthen this observation interval; no queued=1 observation is required.",
    "Camera receipt/header timestamps are not exposure times. Selected thumbnails are observations, not visual odometry or automatic motion validation.",
    "Plots subsample long traces for display and may omit brief peaks. Euler unwrap and joint-position baselines reset per bag/data gap; no cross-gap continuity is inferred.",
]


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def naive_epoch(value, zone):
    """Reject ambiguous/nonexistent local times instead of guessing the DST fold."""
    naive = datetime.fromisoformat(value.replace(",", "."))
    if naive.tzinfo is not None:
        return naive.timestamp()
    candidates = set()
    for fold in (0, 1):
        aware = naive.replace(tzinfo=zone, fold=fold)
        stamp = aware.timestamp()
        if datetime.fromtimestamp(stamp, zone).replace(tzinfo=None) == naive:
            candidates.add(stamp)
    if len(candidates) != 1:
        raise ValueError("ambiguous or nonexistent naive local timestamp")
    return candidates.pop()


def parse_log(text, source, zone):
    events, untimed, gaps = [], [], []
    for number, raw in enumerate(text.splitlines(), 1):
        line = ANSI.sub("", raw)
        loc = {"source": source, "line": number}
        if "job_event " in line:
            try:
                row = json.loads(line.split("job_event ", 1)[1])
                if not isinstance(row, dict): raise ValueError("job event is not an object")
                if row.get("schema") != "pupper_job_event_v1":
                    raise ValueError("unknown job schema")
                required = (isinstance(row.get("job_id"), str) and isinstance(row.get("server_instance"), str)
                            and type(row.get("event_seq")) is int and finite(row.get("wall_time_s"))
                            and finite(row.get("monotonic_s")) and type(row.get("execution_started")) is bool
                            and row.get("event") in {"queued", "started", "cancel_requested", "finished"})
                if not required:
                    raise ValueError("incomplete structured job row")
                events.append({**loc, "at": row["wall_time_s"], "kind": "job", "text": f"{row['event']}: {row.get('name', '')}", "job": row})
            except (ValueError, TypeError) as error:
                gaps.append({**loc, "problem": f"unparsed job event: {error}"})
            continue
        match = API.match(line)
        if match:
            try:
                stamp = naive_epoch(match[1], zone)
                events.append({**loc, "at": stamp, "kind": "api_log", "text": match[4], "timestamp_basis": "naive API log converted with explicit timezone"})
            except ValueError as error:
                gaps.append({**loc, "problem": str(error)})
        else:
            match = NATIVE.search(line)
            if match and EVENT_WORDS.search(match[3]):
                events.append({**loc, "at": float(match[1]), "kind": "native", "text": f"{match[2]}: {match[3]}", "timestamp_basis": "native log epoch"})
            elif re.search(r'"(?:GET|POST|PUT|DELETE) /', line):
                untimed.append({**loc, "text": line, "at": None})
    return events, untimed, gaps


def job_timings(events):
    groups = defaultdict(list)
    for event in events:
        if event["kind"] == "job":
            row = event["job"]
            groups[(row["server_instance"], row["job_id"])].append(row)
    output = []
    for (server, job), rows in sorted(groups.items()):
        by_type = defaultdict(list)
        for row in rows:
            by_type[row["event"]].append(row)
        issues = []
        if len({r["event_seq"] for r in rows}) != len(rows):
            issues.append("duplicate event_seq")
        for name in ("queued", "started", "finished"):
            if len(by_type[name]) > 1:
                issues.append(f"multiple {name} events")
        ordered = sorted(rows, key=lambda r: r["event_seq"])
        if any(b["monotonic_s"] < a["monotonic_s"] for a, b in zip(ordered, ordered[1:])):
            issues.append("monotonic time decreases in event order")
        single = {name: values[0] if len(values) == 1 else None for name, values in by_type.items()}
        queued, started, finished = (single.get(x) for x in ("queued", "started", "finished"))
        if not queued:
            issues.append("queued event missing or ambiguous")
        if not finished:
            issues.append("terminal event missing or ambiguous")
        if finished and finished["execution_started"] and not started:
            issues.append("execution claimed but started event absent or ambiguous")
        if finished and not finished["execution_started"] and started:
            issues.append("terminal execution_started conflicts with started event")
        if queued and finished and (finished["event_seq"] <= queued["event_seq"] or finished["monotonic_s"] < queued["monotonic_s"]):
            issues.append("terminal event precedes queued event")
        def elapsed(a, b):
            if issues or not a or not b:
                return None
            if b["event_seq"] <= a["event_seq"] or b["monotonic_s"] < a["monotonic_s"]:
                issues.append("invalid phase order")
                return None
            return b["monotonic_s"] - a["monotonic_s"]
        queue_wait = elapsed(queued, started)
        execution = elapsed(started, finished)
        if issues:
            queue_wait = execution = None
        output.append({"server_instance": server, "job_id": job, "events": ordered,
                       "queue_wait_s": queue_wait, "execution_s": execution,
                       "duration_clock": "per-server monotonic", "issues": issues,
                       "cancel_before_start": bool(not issues and finished and finished.get("outcome") == "cancelled" and not finished["execution_started"])})
    return output


def historical_correlations(events):
    """Offer bounded name matches only; never infer an ID or a start time."""
    result = []
    for event in events:
        if event["kind"] != "request_record":
            continue
        request = event.get("record", {})
        body = request.get("body", {})
        names = [body.get(k) for k in ("animation_name", "action", "name") if isinstance(body.get(k), str)]
        matches = [other for other in events if other["kind"] == "api_log"
                   and 0 <= other["at"] - event["at"] <= 180
                   and any(re.search(r"(?<!\w)" + re.escape(name) + r"(?!\w)", other["text"]) for name in names)]
        if matches:
            result.append({"request_record": {k: event[k] for k in ("source", "line", "at")},
                           "candidate_log_rows": [{k: other[k] for k in ("source", "line", "at", "text")} for other in matches],
                           "basis": "name plus subsequent180-second window only; no IDs, causal link, or execution-time claim"})
    return result


def action_record(row):
    """Retain route and bounded identifiers, never arbitrary arguments or replies."""
    path = row.get("path", "")
    path = path if isinstance(path, str) and re.fullmatch(r"/[A-Za-z0-9_/-]{1,100}", path) else "unknown route"
    safe = {"path": path, "body": {}}
    if "unlock" not in path and isinstance(row.get("body"), dict):
        for key in ("animation_name", "action", "name"):
            value = row["body"].get(key)
            if isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", value):
                safe["body"][key] = value
    response = row.get("response", {})
    if isinstance(response, dict) and type(response.get("success")) is bool:
        safe["response_success"] = response["success"]
    return safe


def meaningful_status(topic, raw):
    """Collapse periodic timestamps/frame counters; preserve actual state changes."""
    if topic.endswith("/clip_status"):
        match = re.fullmatch(r"(\S+) (playing|holding|stopped) frame \d+ queued \d+ execution (\S+) request (\S+)", raw)
        return f"clip={match[1]} phase={match[2]} execution={match[3]} request={match[4]}" if match else "unrecognized clip status format (raw retained in bag)"
    try:
        value = json.loads(raw)
        if not isinstance(value, dict): raise ValueError("not an object")
        keys = ("protocol", "instance_id", "epoch", "latched", "fault", "stop_required", "in_flight", "unknown_remote_submission", "shutting_down")
        return json.dumps({key: value[key] for key in keys if key in value}, sort_keys=True)
    except (ValueError, TypeError):
        return "unrecognized stop status format (raw retained in bag)"


def clip_record(topic, raw, log_time_ns, index):
    """Retain every wire message; only the explicit native v1 IDs authorize matching."""
    row = {"topic": topic, "controller": topic.rsplit("/", 1)[0], "raw": raw,
           "log_time_ns": log_time_ns, "record_index": index,
           "kind": "request" if topic.endswith("/clip") else "status"}
    parts = raw.split()
    if row["kind"] == "request":
        if (raw.startswith("pupper_clip_") and len(parts) == 3 and parts[0] == "pupper_clip_v1"
                and CLIP_ID.fullmatch(parts[1])):
            row.update(request_id=parts[1], clip=parts[2])
        else:
            row["issue"] = "legacy or malformed request: no usable explicit request ID"
    elif (len(parts) == 10 and parts[1] in {"playing", "holding", "stopped"}
          and parts[2] == "frame" and parts[4] == "queued"
          and parts[6] == "execution" and parts[8] == "request"
          and parts[3].isdigit() and parts[5].isdigit()
          and all(value == "none" or CLIP_ID.fullmatch(value) for value in (parts[7], parts[9]))):
        row.update(clip=parts[0], phase=parts[1], frame=int(parts[3]), queued=int(parts[5]),
                   execution_id=None if parts[7] == "none" else parts[7],
                   request_id=None if parts[9] == "none" else parts[9])
    else:
        row["issue"] = "legacy or malformed status: explicit execution/request IDs unavailable"
    return row


def clip_admission_timings(records):
    """One bag only. Request IDs bind to a unique explicitly reported execution ID.

    Native request IDs change when a queued clip is selected, not when enqueued,
    and persist through automatic next/loop/hold transitions. An endpoint or a
    different clip as the first same-ID observation cannot locate admission.
    """
    requests, statuses = defaultdict(list), defaultdict(list)
    for row in records:
        if row.get("request_id"):
            target = requests if row["kind"] == "request" else statuses
            target[(row["controller"], row["request_id"])].append(row)
    result = []
    for controller, request_id in sorted(requests.keys() | statuses.keys()):
        key = controller, request_id
        sent = requests.get(key, [])
        observed = sorted(statuses.get(key, []), key=lambda r: (r["log_time_ns"], r["record_index"]))
        issues = []
        if len(sent) != 1:
            issues.append("request missing" if not sent else "multiple request records: duplicate or replay is ambiguous")
        executions = {r.get("execution_id") for r in observed}
        execution = next(iter(executions)) if len(executions) == 1 and None not in executions else None
        if not observed:
            issues.append("matching applied status missing")
        elif execution is None:
            issues.append("execution ID missing or multiple executions share this request ID")
        first = observed[0] if observed else None
        if len(sent) == 1 and first:
            if (first["log_time_ns"], first["record_index"]) < (sent[0]["log_time_ns"], sent[0]["record_index"]):
                issues.append("same-ID status precedes request: retained, replayed or receipt order is unresolved")
            if first["clip"] != sent[0]["clip"] or first["phase"] != "playing":
                issues.append("first same-ID status is not playing the requested clip: endpoint or earlier application unobserved")
        if execution and len(observed) > 1:
            # An ID can persist through next/loop transitions, but cannot return
            # after another selected request without replay or record ambiguity.
            lo = first["log_time_ns"], first["record_index"]
            hi = observed[-1]["log_time_ns"], observed[-1]["record_index"]
            if any(r["kind"] == "status" and r["controller"] == controller
                   and r.get("execution_id") == execution and r.get("request_id") != request_id
                   and lo < (r["log_time_ns"], r["record_index"]) < hi for r in records):
                issues.append("request ID reappears after a different selected request")
        result.append({"controller": controller, "request_id": request_id, "execution_id": execution,
                       "requested_clip": sent[0]["clip"] if len(sent) == 1 else None,
                       "request_record_indices": [r["record_index"] for r in sent],
                       "matching_status_count": len(observed),
                       "first_observed_status_record_index": first["record_index"] if first else None,
                       "request_to_first_observed_applied_s":
                           (first["log_time_ns"] - sent[0]["log_time_ns"]) / 1e9 if not issues else None,
                       "issues": issues})
    return {"clock": "same-bag MCAP log_time receipts; not physical latency",
            "available": any(r["request_to_first_observed_applied_s"] is not None for r in result),
            "issues": [] if requests else ["no versioned clip requests recorded; admission timing unavailable"],
            "unidentified_record_indices": [r["record_index"] for r in records if not r.get("request_id")],
            "requests": result}


def overlap_records(bags):
    overlaps = []
    for index, a in enumerate(bags):
        for b in bags[index + 1:]:
            for topic in sorted(set(a.get("topics", {})) & set(b.get("topics", {}))):
                aa, bb = a["topics"][topic], b["topics"][topic]
                if aa.get("start") is None or bb.get("start") is None:
                    continue
                lo, hi = max(aa["start"], bb["start"]), min(aa["end"], bb["end"])
                if lo <= hi:
                    overlaps.append({"bags": [a["source"], b["source"]], "topic": topic,
                                     "start": lo, "end": hi, "treatment": "kept separate; no timestamp deduplication"})
    return overlaps


def topic_coverage(bags, start, end):
    output = {}
    for topic in (IMU, JOINTS, *sorted(COMMANDS)):
        ranges = sorted((b['topics'][topic]['start'], b['topics'][topic]['end']) for b in bags if topic in b['topics'])
        merged = []
        for lo, hi in ranges:
            if merged and lo <= merged[-1][1]: merged[-1][1] = max(hi, merged[-1][1])
            else: merged.append([lo, hi])
        cursor, gaps = start, []
        for lo, hi in merged:
            if lo > cursor: gaps.append([cursor, lo])
            cursor = max(cursor, hi)
        if cursor < end: gaps.append([cursor, end])
        output[topic] = {'bag_receipt_extents': merged, 'outside_recorded_extents': gaps,
                         'meaning': 'coverage extents only; gaps inside a recording are reported separately; silence is not a held-zero command'}
    return output


def numeric_stats(values):
    import numpy as np
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return {"min": float(a.min()), "median": float(np.median(a)), "p99": float(np.quantile(a, .99)), "max": float(a.max())} if a.size else None


def read_bag(path, relative):
    import numpy as np
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    info = {"source": relative, "topics": {}, "invalid": {}, "error": None, "crc_validation_requested": True}
    streams = defaultdict(list)
    names = None
    state_events = []
    clip_records = []
    with path.open("rb") as stream:
        reader = make_reader(stream, validate_crcs=True, decoder_factories=[DecoderFactory()])
        try:
            summary = reader.get_summary()
            info["has_summary"] = summary is not None
            # Without summary channels, discover topics from raw records first.
            # Decoding all channels before filtering would fail on unrelated
            # non-ROS schemas. The seekable reader resets for the selected pass.
            channels = (summary.channels.values() if summary and summary.channels else
                        (channel for _, channel, _ in reader.iter_messages(log_time_order=False)))
            topics = TOPICS | {c.topic for c in channels
                              if c.topic.endswith("/clip_status") or CLIP_TOPIC.fullmatch(c.topic)}
            for _, channel, message, data in reader.iter_decoded_messages(topics=sorted(topics)):
                topic, stamp = channel.topic, message.log_time / 1e9
                if topic not in TOPICS and not topic.endswith("/clip_status") and not CLIP_TOPIC.fullmatch(topic):
                    continue
                rowinfo = info["topics"].setdefault(topic, {"messages": 0, "start": stamp, "end": stamp})
                rowinfo["messages"] += 1
                rowinfo["start"], rowinfo["end"] = min(rowinfo["start"], stamp), max(rowinfo["end"], stamp)
                try:
                    if topic == IMU:
                        q, g = data.orientation, data.angular_velocity
                        row = [stamp, data.header.stamp.sec + data.header.stamp.nanosec / 1e9,
                               q.x, q.y, q.z, q.w, g.x, g.y, g.z]
                        if not np.isfinite(row).all() or np.linalg.norm(row[2:6]) < 1e-8:
                            raise ValueError("nonfinite or zero quaternion IMU")
                        streams[topic].append(row)
                    elif topic == JOINTS:
                        current = list(data.name)
                        if not current or len(current) != len(set(current)):
                            raise ValueError("missing or duplicate joint names")
                        if names is None:
                            names = sorted(current)
                        if set(current) != set(names) or len(data.position) != len(current):
                            raise ValueError("joint names/positions changed or incomplete")
                        order = [current.index(n) for n in names]
                        positions = [data.position[i] for i in order]
                        speed = max(abs(x) for x in data.velocity) if len(data.velocity) == len(current) else math.nan
                        if not np.isfinite(positions).all():
                            raise ValueError("nonfinite joint position")
                        streams[topic].append([stamp, *positions, speed])
                    elif topic in COMMANDS:
                        row = [stamp, data.linear.x, data.linear.y, data.angular.z]
                        if not np.isfinite(row).all():
                            raise ValueError("nonfinite command")
                        streams[topic].append(row)
                    elif topic == "/emergency_stop":
                        state_events.append({"at": stamp, "kind": "telemetry_event", "source": relative, "text": topic + " message received (payload not interpreted)"})
                    else:
                        if CLIP_TOPIC.fullmatch(topic):
                            clip_records.append(clip_record(topic, str(data.data), message.log_time, len(clip_records)))
                        if topic.endswith("/clip"):
                            continue  # Retained separately; no periodic-status collapse or job inference.
                        value = meaningful_status(topic, str(data.data))
                        if not streams[topic] or value != streams[topic][-1][1]:
                            streams[topic].append([stamp, value])
                            state_events.append({"at": stamp, "kind": "telemetry_event", "source": relative, "text": topic + ": " + value})
                except (AttributeError, ValueError, IndexError, TypeError) as error:
                    key = f"{topic}: {error}"
                    info["invalid"][key] = info["invalid"].get(key, 0) + 1
        except Exception as error:
            info["error"] = f"{type(error).__name__}: {error}"
    numeric = {k: np.asarray(v, dtype=float) for k, v in streams.items() if k in {IMU, JOINTS, *COMMANDS}}
    info["joint_names"] = names
    info["clip_records"] = clip_records
    info["clip_admission"] = clip_admission_timings(clip_records)
    info["status_transitions"] = {topic: len(values) for topic, values in streams.items() if topic not in {IMU, JOINTS, *COMMANDS}}
    for topic, values in numeric.items():
        delta = np.diff(values[:, 0])
        gaps = np.flatnonzero(delta > 1.)
        info["topics"][topic].update({"valid_samples": len(values), "receipt_gap_s": numeric_stats(delta),
                                     "nonincreasing_receipt_steps": int((delta <= 0).sum()),
                                     "receipt_gaps_over_1s": [[float(values[i, 0]), float(values[i+1, 0])] for i in gaps]})
    if JOINTS in numeric:
        info['reported_max_absolute_joint_velocity_rad_s'] = numeric_stats(numeric[JOINTS][:, -1])
    if IMU in numeric:
        values = numeric[IMU]
        info["imu_legacy_header_minus_receipt_s"] = numeric_stats(values[:, 1] - values[:, 0])
        info["imu_quaternion_norm"] = numeric_stats(np.linalg.norm(values[:, 2:6], axis=1))
    for topic in COMMANDS & numeric.keys():
        info["topics"][topic]["nonzero_messages"] = int(np.any(abs(numeric[topic][:, 1:]) > 1e-8, axis=1).sum())
    return info, numeric, state_events


def safe_frame_path(root, metadata, filename):
    if not isinstance(filename, str) or Path(filename).is_absolute():
        raise ValueError("frame path is not relative")
    path = (metadata.parent / filename).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_relative_to(metadata.parent.resolve()):
        raise ValueError("frame path escapes camera directory")
    return path


def read_camera(root, metadata, register, gaps):
    frames = []
    for line, raw in enumerate(metadata.read_text().splitlines(), 1):
        try:
            row = json.loads(raw)
            if not isinstance(row, dict): raise ValueError("camera row is not an object")
            if not row.get("file"):
                continue
            if not finite(row.get("arrival_wall_s")):
                raise ValueError("saved frame lacks wall receipt time")
            path = safe_frame_path(root, metadata, row["file"])
            if not path.is_file():
                raise ValueError("saved frame missing")
            frames.append({"at": row["arrival_wall_s"], "path": path,
                           "source": str(metadata.relative_to(root)), "line": line,
                           "header_sec": row.get("header_sec"), "header_nanosec": row.get("header_nanosec"),
                           "declared_sha256": row.get("sha256")})
        except (ValueError, TypeError) as error:
            gaps.append({"source": str(metadata.relative_to(root)), "line": line, "problem": str(error)})
    return frames


def select_frames(groups, count):
    import numpy as np
    groups = [sorted(g, key=lambda x: x["at"]) for g in groups if g]
    selected = []
    for i, group in enumerate(groups):
        number = min(len(group), count // len(groups) + (i < count % len(groups)))
        selected.extend(group[index] for index in sorted(set(np.linspace(0, len(group) - 1, number, dtype=int))))
    return sorted(selected, key=lambda x: x["at"])


def plot_timeline(series, events, t0, end):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(6, 1, figsize=(13, 15), sharex=True, constrained_layout=True)
    kinds = {"request_record": 0, "job": 1, "api_log": 2, "native": 3, "telemetry_event": 4, "camera_thumbnail": 5}
    for kind, lane in kinds.items():
        times = [e["at"] - t0 for e in events if e["kind"] == kind]
        if times:
            axes[0].scatter(times, [lane] * len(times), s=10, marker="|")
    requests = [e for e in events if e['kind'] == 'request_record']
    if len(requests) <= 12:
        for event in requests:
            axes[0].annotate(event['text'][:70], (event['at']-t0, 0), xytext=(3, 8), textcoords='offset points', fontsize=6, rotation=25)
    axes[0].set_yticks(list(kinds.values()), list(kinds)); axes[0].set_title("Recorded events and reported signals — common host-epoch axis")
    palette = ["#2563eb", "#d97706", "#059669"]
    def chunks(a):
        # No line or Euler unwrap is allowed to bridge a long data gap.
        d = np.diff(a[:, 0]); positive = d[d > 0]
        threshold = max(1., 10 * float(np.median(positive))) if len(positive) else 1.
        return np.split(a, np.where((d <= 0) | (d > threshold))[0] + 1)
    for bag_index, (name, data) in enumerate(series):
        style = ["-", "--", ":", "-."][bag_index % 4]
        if IMU in data:
            for segment in chunks(data[IMU]):
                q = segment[:, 2:6]; q = q / np.linalg.norm(q, axis=1)[:, None]
                x, y, z, w = q.T
                angles = np.c_[np.unwrap(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
                               np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                               np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]
                stride = max(1, len(segment) // 12000)
                for k, label in enumerate(("roll", "pitch", "yaw")):
                    axes[1].plot(segment[::stride, 0]-t0, np.degrees(angles[::stride, k]), color=palette[k], ls=style, lw=.7, label=f"{bag_index+1} {label}")
                    axes[2].plot(segment[::stride, 0]-t0, segment[::stride, 6+k], color=palette[k], ls=style, lw=.7, label=f"{bag_index+1} {'xyz'[k]} axis")
        for topic in COMMANDS & data.keys():
            a = data[topic]
            # Points only: publishing gaps do not establish a held command.
            for k, label in enumerate(("vx m/s", "vy m/s", "wz rad/s")):
                axes[3].plot(a[:, 0]-t0, a[:, k+1], ".", ms=2, color=palette[k], label=f"{bag_index+1} {topic} {label}")
        if JOINTS in data:
            for a in chunks(data[JOINTS]):
                stride = max(1, len(a) // 12000)
                position = np.sqrt(np.mean((a[:, 1:-1]-a[0, 1:-1])**2, axis=1))
                axes[4].plot(a[::stride, 0]-t0, position[::stride], ls=style, lw=.8, label=f"bag {bag_index+1}")
                axes[5].plot(a[::stride, 0]-t0, a[::stride, -1], ls=style, lw=.8, label=f"bag {bag_index+1}")
    for axis, label in zip(axes[1:], ("Reported RPY (deg)", "Reported body gyro (rad/s)", "Command samples (mixed units)", "Joint RMS Δposition (rad)*", "Reported max |joint velocity| (rad/s)")):
        axis.set_ylabel(label); axis.grid(alpha=.2)
        handles, labels = axis.get_legend_handles_labels()
        dedup = dict(zip(labels, handles))
        if dedup: axis.legend(dedup.values(), dedup.keys(), loc="upper left", fontsize=7, ncol=3)
    axes[-1].set_xlabel("Elapsed seconds from report origin; *position baseline resets per bag/gap")
    axes[-1].set_xlim(0, max(1., end-t0))
    stream = io.BytesIO(); fig.savefig(stream, format="png", dpi=120); plt.close(fig)
    return stream.getvalue()


def build_report(root, output, zone_name, camera_count=12):
    from PIL import Image
    root, output = Path(root).resolve(), Path(output).resolve()
    zone = ZoneInfo(zone_name)
    if not root.is_dir() or output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("input must be a directory; output must be outside it")
    output.mkdir(parents=True, exist_ok=False)
    inputs, gaps, events, untimed, bags, series = {}, [], [], [], [], []
    def register(path):
        path = path.resolve()
        if not path.is_relative_to(root):
            raise ValueError("input symlink escapes run directory")
        key = str(path.relative_to(root))
        if key not in inputs:
            inputs[key] = {"bytes": path.stat().st_size, "sha256": sha(path), "mtime_ns": path.stat().st_mtime_ns}
        return key
    seen_bags = {}
    provenance = []
    for name in ("runtime-manifest.json", "owned-processes.json", "control-shutdown.json", "data-shutdown.json", "deployment-inputs/manifest.json", "frozen-inputs-index.json"):
        path = root / name
        if path.is_file(): provenance.append(register(path))
    passive = root / "post-disable-passive"
    if passive.is_dir():
        omitted = [register(path) for path in sorted(passive.glob("*.jsonl"))]
        gaps.append({"problem": "post-disable-passive raw sensor packets are not decoded by this MCAP report; use the separate sensor analysis", "sources": omitted})
    for path in sorted(root.rglob("*.mcap")):
        name = register(path); digest = inputs[name]["sha256"]
        if digest in seen_bags:
            gaps.append({"source": name, "problem": "byte-identical duplicate bag skipped", "identical_to": seen_bags[digest]})
            continue
        seen_bags[digest] = name
        info, data, states = read_bag(path, name)
        bags.append(info); series.append((name, data)); events.extend(states)
        if info["error"]:
            gaps.append({"source": name, "problem": "partial bag read: " + info["error"]})
        metadata = path.parent / "metadata.yaml"
        if metadata.is_file(): register(metadata)
    # Primary session logs only; roslogs often duplicate launch.log verbatim.
    for name in ("api.log", "launch.log"):
        path = root / name
        if path.is_file():
            register(path); parsed, no_time, errors = parse_log(path.read_text(errors="replace"), name, zone)
            events.extend(parsed); untimed.extend(no_time); gaps.extend(errors)
        else:
            gaps.append({"source": name, "problem": "primary log absent"})
    for name in ("operator-actions.jsonl", "direct-tests.jsonl"):
        path = root / name
        if not path.is_file(): continue
        register(path)
        for number, raw in enumerate(path.read_text().splitlines(), 1):
            try:
                row = json.loads(raw)
                if not isinstance(row, dict): raise ValueError("request row is not an object")
                stamp = row.get("at")
                if not finite(stamp): raise ValueError("request record lacks finite at timestamp")
                safe = action_record(row)
                events.append({"at": stamp, "source": name, "line": number, "kind": "request_record", "text": safe['path'] + " " + ", ".join(safe['body'].values()), "record": safe,
                               "timestamp_basis": "recorded at field; request start/receipt meaning not independently established"})
            except (ValueError, TypeError) as error:
                gaps.append({"source": name, "line": number, "problem": str(error)})
    frames = []
    for metadata in sorted(root.rglob("frames.jsonl")):
        register(metadata); frames.append(read_camera(root, metadata, register, gaps))
    thumbnails = []
    for frame in select_frames(frames, camera_count):
        path = frame.pop("path"); name = register(path); digest = inputs[name]["sha256"]
        if frame["declared_sha256"] and frame["declared_sha256"] != digest:
            gaps.append({"source": name, "problem": "camera declared hash mismatch; thumbnail omitted"}); continue
        try:
            with Image.open(path) as image:
                image.thumbnail((480, 360)); stream = io.BytesIO(); image.convert("RGB").save(stream, format="JPEG", quality=80)
            thumbnails.append({**frame, "image_source": name, "sha256": digest, "jpeg_base64": base64.b64encode(stream.getvalue()).decode()})
        except (ValueError, OSError) as error:
            gaps.append({"source": name, "problem": f"thumbnail decode failed: {error}"})
    timestamps = [e["at"] for e in events] + [f["at"] for group in frames for f in group]
    timestamps += [entry[key] for bag in bags for entry in bag["topics"].values() for key in ("start", "end")]
    t0 = min(timestamps) if timestamps else None
    if t0 is None:
        gaps.append({"problem": "no usable epoch timestamps"}); t0 = 0.
    events.sort(key=lambda e: (e["at"], e["source"], e.get("line", 0)))
    events.extend({'at': frame['at'], 'kind': 'camera_thumbnail', 'source': frame['image_source'], 'text': 'Selected saved camera frame'} for frame in thumbnails)
    events.sort(key=lambda e: (e['at'], e['source'], e.get('line', 0)))
    for event in events: event["elapsed_s"] = event["at"] - t0
    for frame in thumbnails: frame["elapsed_s"] = frame["at"] - t0
    for topic in (IMU, JOINTS, "/cmd_vel"):
        if not any(topic in bag["topics"] for bag in bags): gaps.append({"problem": f"required display topic absent: {topic}"})
    if not thumbnails: gaps.append({"problem": "no verified timestamped camera thumbnail available"})
    end = max(timestamps) if timestamps else t0
    plot = plot_timeline(series, events, t0, end)
    (output / "timeline.png").write_bytes(plot)
    changed = [name for name, info in inputs.items() if sha(root / name) != info["sha256"]]
    if changed: gaps.append({"problem": "inputs changed during report; not a closed snapshot", "sources": changed})
    versions = {}
    for package in ("numpy", "matplotlib", "mcap", "mcap-ros2-support", "Pillow"):
        try: versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: versions[package] = "unknown"
    report = {"schema": "pupper_closed_session_report_v1", "input": str(root), "source_sha256": sha(Path(__file__)),
              "generated_at_utc": datetime.now(timezone.utc).isoformat(), "naive_api_timezone": zone_name,
              "origin_epoch_s": t0, "end_epoch_s": max(timestamps) if timestamps else None,
              "input_unchanged_after_read": not changed, "inputs": inputs, "packages": versions,
              "runtime_provenance_files": provenance,
              "bags": bags, "bag_overlaps": overlap_records(bags), "events": events, "untimed_http_access": untimed,
              "topic_coverage": topic_coverage(bags, t0, end),
              "structured_jobs": job_timings(events), "historical_candidate_correlations": historical_correlations(events),
              "camera_thumbnails": [{k: v for k, v in f.items() if k != "jpeg_base64"} for f in thumbnails],
              "data_gaps": gaps, "limitations": LIMITS, "plot_sha256": hashlib.sha256(plot).hexdigest()}
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    esc = lambda value: html.escape(str(value))
    rows = "".join(f"<tr><td>{e['elapsed_s']:.3f}</td><td>{esc(e['kind'])}</td><td>{esc(e['text'])}</td><td>{esc(e['source'])}:{e.get('line', '')}</td></tr>" for e in events)
    photos = "".join(f"<figure><img src='data:image/jpeg;base64,{f['jpeg_base64']}'><figcaption>+{f['elapsed_s']:.3f}s · {esc(f['image_source'])}<br>Host receipt {f['at']:.6f}; header {f['header_sec']}.{str(f['header_nanosec']).zfill(9)}</figcaption></figure>" for f in thumbnails)
    summary = f"{len(bags)} bags · {len(events)} timed events · {len(untimed)} untimed HTTP rows · {len(report['structured_jobs'])} structured jobs · {len(thumbnails)} thumbnails"
    markup = f"""<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width'><title>Lab 6 closed-session report</title><style>body{{font:16px system-ui;max-width:1400px;margin:auto;padding:24px;color:#17212b}}img{{max-width:100%}}table{{border-collapse:collapse;width:100%;font-size:13px}}td,th{{padding:7px;text-align:left;border-bottom:1px solid #ddd;vertical-align:top}}.photos{{display:flex;flex-wrap:wrap}}figure{{margin:10px;width:31%}}figcaption{{font-size:12px;overflow-wrap:anywhere}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}.note{{background:#fff4d6;padding:14px}}details{{margin:20px 0}}@media(max-width:700px){{figure{{width:100%}}}}</style></head><body>
<h1>Lab 6 closed-session report</h1><p>{esc(summary)}</p><p>Origin: {esc(datetime.fromtimestamp(t0, timezone.utc).isoformat())}; naive API log timezone: {esc(zone_name)}.</p><p class='note'>Reported signals and software events only. This report does not establish physical success or the cause of drift/falls.</p>
<img alt='Common elapsed timeline' src='data:image/png;base64,{base64.b64encode(plot).decode()}'>
<h2>Selected camera observations</h2><div class='photos'>{photos}</div>
<h2>Timed evidence</h2><table><thead><tr><th>Elapsed s</th><th>Kind</th><th>Recorded event</th><th>Source</th></tr></thead><tbody>{rows}</tbody></table>
<details open><summary>Data gaps and overlapping recordings</summary><pre>{esc(json.dumps({'gaps':gaps,'overlaps':report['bag_overlaps'],'topic_coverage':report['topic_coverage']},indent=2))}</pre></details>
<details><summary>Structured job timings and historical candidate correlations</summary><pre>{esc(json.dumps({'jobs':report['structured_jobs'],'historical_candidates':report['historical_candidate_correlations']},indent=2))}</pre></details>
<details><summary>Clip request-to-applied-status receipt intervals (not physical latency)</summary><pre>{esc(json.dumps([{'source':b['source'],**b['clip_admission']} for b in bags],indent=2))}</pre><p>Every clip request and status, including repeated statuses, is retained in report.json under each bag's clip_records. Indices above refer only to that bag.</p></details>
<details><summary>Untimed HTTP rows — no time assigned</summary><pre>{esc(json.dumps(untimed,indent=2))}</pre></details>
<details><summary>Input identities and package versions</summary><pre>{esc(json.dumps({'inputs':inputs,'packages':versions,'source_sha256':report['source_sha256']},indent=2))}</pre></details>
<h2>Interpretation limits</h2><ul>{''.join('<li>'+esc(x)+'</li>' for x in LIMITS)}</ul></body></html>"""
    (output / "report.html").write_text(markup)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("closed_run", type=Path)
    parser.add_argument("new_output", type=Path)
    parser.add_argument("--timezone", required=True, help="IANA timezone for naive API timestamps, e.g. America/Los_Angeles")
    parser.add_argument("--camera-thumbnails", type=int, default=12, choices=range(0, 25), metavar="0..24")
    args = parser.parse_args()
    try:
        report = build_report(args.closed_run, args.new_output, args.timezone, args.camera_thumbnails)
    except (ValueError, OSError) as error:
        parser.exit(2, f"Report failed: {error}\n")
    print(json.dumps({"output": str(args.new_output.resolve()), "bags": len(report['bags']), "events": len(report['events']), "gaps": len(report['data_gaps']), "inputs_unchanged": report['input_unchanged_after_read']}))
    return 0 if report["input_unchanged_after_read"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
