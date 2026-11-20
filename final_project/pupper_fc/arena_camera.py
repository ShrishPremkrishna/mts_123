"""Own the overhead device once and publish one measured field frame to the selected match."""
from __future__ import annotations

import base64
import math
import threading
import time
import uuid


def camera_settings(data, profile):
    mode = data.get("field_mode", "layout")
    tracking = data.get("robot_tracking", "tags")
    if mode not in ("layout", "corner") or tracking not in ("tags", "colors"):
        raise ValueError("Select a field reference and robot tracking method.")
    setup = {"mode": mode}
    if tracking == "colors":
        height = data.get("patch_height", .18)
        if type(height) not in (int, float) or not math.isfinite(height) or not .03 <= height <= .60:
            raise ValueError("Enter the height of both patch surfaces above the floor between 0.03 and 0.60 metres.")
        setup["patch_height"] = float(height)
    if mode == "corner":
        anchor = data.get("anchor")
        if not isinstance(anchor, dict):
            raise ValueError("Enter the measurements for the selected corner tag.")
        corner, tid = anchor.get("corner"), anchor.get("tag_id")
        if not isinstance(corner, str) or corner not in profile["corner_ids"] or type(tid) is not int or not 0 <= tid < 587 or tid in (10, 11):
            raise ValueError("Select a field corner and a unique floor ID from 0 to 586 (10 and 11 are robot IDs).")
        setup.update(corner=corner, tag_id=tid)
        for key in ("length", "width", "tag_size", "rotation", "inset_x", "inset_y", "goal_width"):
            v = anchor.get(key)
            if type(v) not in (float, int) or not math.isfinite(v):
                raise ValueError("Enter a finite number for " + key.replace("_", " ") + ".")
            setup[key] = float(v)
        if not (.5 <= setup["length"] <= 15 and .5 <= setup["width"] <= 10
                and 2 * profile["field"]["ball_radius"] < setup["goal_width"] <= setup["width"]
                and .04 <= setup["tag_size"] <= min(setup["length"], setup["width"])
                and abs(setup["rotation"]) <= 360
                and abs(setup["inset_x"]) < setup["length"] / 2 and abs(setup["inset_y"]) < setup["width"] / 2):
            raise ValueError("Check dimensions in metres, centre offsets, black-square side, mouth width and rotation in degrees.")
    return setup, tracking


class ArenaCamera:
    def __init__(self, profile, source="phone", *, appearance=False):
        self.profile, self.source, self.appearance = profile, source, appearance
        self.session = uuid.uuid4().hex
        self.thread = None
        self.done = threading.Event()
        self.lock = threading.RLock()
        self.director = None
        self.publishers = ()
        self.picture = None
        self.preview_meta = self.frozen = self.confirmed_revision = None
        self.preview_gray = self.frozen_gray = self.far_corner = None
        self.goal_lines = profile.get("surveyed_goals")
        self.state = {"status": "Disconnected", "source_session": self.session, "live": False,
                      "frame": None, "visible": {}, "match": None, "appearance": []}

    def start(self):
        with self.lock:
            if self.thread is not None:
                raise ValueError("Camera is already attached")
            self.state["status"] = "Opening camera"
            self.thread = threading.Thread(target=self._run, name="fc-field-camera", daemon=True)
            self.thread.start()

    def close(self):
        self.done.set()
        self.set_match(None)
        if self.thread:
            self.thread.join(timeout=8)
            if self.thread.is_alive():
                raise RuntimeError("Waiting for the camera to release")

    def snapshot(self):
        with self.lock:
            snap = dict(self.state)
        frame = snap["frame"]
        snap["age_s"] = time.time() - frame["captured_at"] if frame and snap["live"] else None
        return snap

    def jpeg(self):
        with self.lock:
            return self.picture

    def capture_setup(self):
        with self.lock:
            if self.director or self.picture is None or self.preview_meta is None or not self.state["frame"]:
                raise ValueError("Open Camera settings while the match is idle and wait for a frame.")
            self.frozen = dict(self.preview_meta, token=uuid.uuid4().hex)
            self.frozen_gray = self.preview_gray
            return dict(self.frozen, image="data:image/jpeg;base64," + base64.b64encode(self.picture).decode("ascii"))

    def mark_far_corner(self, data):
        from final_project.overhead_perception.fieldcam.corner_anchor import FarCorner
        with self.lock:
            if (self.director or not self.frozen or self.frozen_gray is None or not self.state["frame"]
                    or self.frozen["token"] != data.get("token") or self.frozen["mode"] != "corner"
                    or self.state["frame"]["calibration_revision"] != self.frozen["revision"]
                    or self.profile["field_setup"]["tag_id"] not in self.frozen["ids"]):
                raise ValueError("Take a new corner-tag snapshot with the reference square in view.")
            p = data.get("point")
            if not (isinstance(p, list) and len(p) == 2 and all(type(v) in (int, float) and 0 <= v <= 1 for v in p)):
                raise ValueError("Click the opposite physical corner in the snapshot.")
            h, w = self.frozen_gray.shape
            self.far_corner = FarCorner(self.frozen_gray, [p[0]*(w-1), p[1]*(h-1)])
            self.confirmed_revision = self.frozen = None

    def _check_snapshot(self, data):
        frame = self.state["frame"] or {}
        if (self.director or not self.frozen or data.get("token") != self.frozen["token"]
                or not self.frozen["fit_valid"] or not self.state.get("fit_valid")
                or self.frozen["revision"] != frame.get("calibration_revision")
                or (self.state["live"] and not 0 <= time.time() - frame["captured_at"] <= .5)):
            raise ValueError("Field view changed. Take a new snapshot and check the reference marks.")

    def confirm_field(self, data):
        with self.lock:
            self._check_snapshot(data)
            if self.profile.get("field_setup", {}).get("mode") != "corner":
                raise ValueError("The distributed layout is checked automatically.")
            self.confirmed_revision = self.frozen["revision"]
            self.frozen = None

    def set_goals(self, goals):
        with self.lock:
            self.goal_lines = goals

    def observe_kick(self, event):
        with self.lock:
            if self.director:
                self.director.observe_kick(event)

    def survey(self, data):
        with self.lock:
            self._check_snapshot(data)
            frame = self.state["frame"] or {}
            if (not self.state["live"] or not frame.get("calibration_valid")
                    or self.session != data.get("session") or frame["calibration_revision"] != data.get("revision")
                    or time.time() - frame["captured_at"] > .5):
                raise ValueError("The camera view changed. Collect the four post positions again.")
            points = data.get("points")
            if not (isinstance(points, list) and len(points) == 4 and all(isinstance(p,list) and len(p)==2
                    and all(type(v) in (int,float) and math.isfinite(v) and 0 <= v <= 1 for v in p) for p in points)):
                raise ValueError("Choose four post positions in the camera preview")
            cam, w, h = self.projection
            xy = cam.pixel_to_plane([[u*(w-1), v*(h-1)] for u,v in points], 0)
            goals = {}
            for side, pair in (("west", xy[:2]), ("east", xy[2:])):
                if abs(pair[0,0]-pair[1,0]) > .10:
                    raise ValueError("Both posts of each goal must share the same end line within 10 cm")
                goals[side] = {"x": float(pair[:,0].mean()), "y_min":float(pair[:,1].min()), "y_max":float(pair[:,1].max())}
            return goals

    def set_match(self, director, publishers=()):
        with self.lock:
            if publishers and (not director or not self.state["live"]):
                raise ValueError("Live play requires a live camera")
            self.director, self.publishers = director, tuple(publishers)

    def resume(self):
        with self.lock:
            if not self.director:
                raise ValueError("No active match")
            return self.director.resume()

    def commit_start(self):
        with self.lock:
            self.director.commit_start()

    def _run(self):
        source = None
        try:
            import cv2
            import numpy as np
            from final_project.overhead_perception.fieldcam.capture import FrameGrabber
            from final_project.overhead_perception.fieldcam.config import SIM_CONFIG, load_config, set_field_size
            from final_project.overhead_perception.fieldcam.pipeline import Perception
            simulation = self.source == "sim"
            cfg = load_config(SIM_CONFIG if simulation else None)
            profile = self.profile
            setup = profile.get("field_setup", {"mode": "layout"})
            cfg.update(field_setup=setup, robot_tracking=profile.get("robot_tracking", "tags"))
            if cfg["robot_tracking"] == "colors":
                for robot in cfg["robots"].values():
                    robot["tag_height"] = setup.get("patch_height", .18)
            if not simulation:
                cfg["camera"].update(profile.get("camera", {}))
                f = profile["field"]
                cfg["field"]["goal_width"], cfg["ball"]["radius"] = f["goal_width"], f["ball_radius"]
                set_field_size(cfg, f["length"], f["width"])
                cfg["tags"]["field"] = {int(i): np.array(p, float) for i, p in profile["tags"].items()}
                cfg["tags"]["corner_ids"] = profile["corner_ids"]
                cfg["tags"]["floor_size"] = profile["floor_size"]
            if setup["mode"] == "corner":
                from final_project.overhead_perception.fieldcam.corner_anchor import anchor_geometry
                cfg["field"]["goal_width"] = setup["goal_width"]
                set_field_size(cfg, setup["length"], setup["width"])
                cfg["tags"]["field"] = {setup["tag_id"]: anchor_geometry(setup).mean(axis=0)}
            if simulation:
                from final_project.overhead_perception.scripts.run_perception import SimSource
                source = SimSource(cfg, (1280, 800))
                if setup["mode"] == "corner":
                    source.robots["striker"][:2] = [-.30 * setup["length"], -.18 * setup["width"]]
                    source.robots["keeper"][:2] = [.30 * setup["length"], .18 * setup["width"]]
            else:
                c = cfg["camera"]
                device = int(self.source) if self.source.isdigit() else self.source
                source = FrameGrabber(device, c["width"], c["height"], c["fps"], wait=2)
            live = not simulation and not source.is_file
            width = min(self.profile.get("processing_width", 896), source.width)
            # A single square needs as much local detail as the sensor supplies.
            if setup["mode"] == "corner":
                width = source.width
            height = round(source.height * width / source.width)
            perception = Perception(cfg, width, height)
            mapped = sorted(cfg["tags"]["field"].items(), key=lambda kv: kv[1][0])
            rows = [[str(tid) for tid, xy in mapped if xy[1] * sign > 0] for sign in (1, -1)]
            guide = f"From west to east: north sideline {', '.join(rows[0])}; south sideline {', '.join(rows[1])}."
            previous_n, retries, next_cal, next_preview, previous_video = -1, 0, 0., 0., None
            while not self.done.is_set():
                frame, receipt, previous_n = source.read(previous_n)
                if frame is None:
                    if not live:
                        break
                    if source.lost_for:
                        perception.invalidate_calibration("camera feed lost")
                        with self.lock:
                            self.state.update(status="Camera feed lost", live=False, frame=None, visible={})
                            self.picture = None
                    continue
                if getattr(source, "reconnects", 0) != retries:
                    retries = source.reconnects
                    perception.invalidate_calibration("camera reconnected")
                    with self.lock:
                        self.far_corner = self.frozen = self.confirmed_revision = None
                if (frame.shape[1], frame.shape[0]) != (source.width, source.height):
                    raise ValueError("Camera resolution changed. Reconnect the camera.")
                if width != source.width:
                    frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
                now = time.monotonic()
                with self.lock:
                    may_calibrate = self.director is None
                    if may_calibrate and perception.far_corner is not self.far_corner:
                        perception.far_corner = self.far_corner
                        perception.invalidate_calibration("Updating opposite-corner reference")
                        next_cal = 0.
                if may_calibrate and not perception.calibration_valid and now >= next_cal:
                    next_cal = now + .4
                    perception.calibrate(frame)
                captured = receipt - cfg["camera"]["latency_s"] if live else receipt
                if not live and not simulation:
                    captured = source.file_time_s(previous_video, None)
                    previous_video = captured
                state = perception.process(frame, captured, time.time() if live else captured, match_only=True)
                fit_valid = state["calibration_valid"]
                with self.lock:
                    needs_confirm = setup["mode"] == "corner" and fit_valid and self.confirmed_revision != state["calibration_revision"]
                    goals = self.goal_lines
                if needs_confirm:
                    state.update(calibration_valid=False, calibrated=False, calibration_status="awaiting_confirmation")
                tags = sorted(perception.debug["tags"])
                visible = {"tags": tags, "ball": state["ball"]["measured_this_frame"],
                           "robots": {n: v["measured_this_frame"] for n, v in state["robots"].items()}}
                status = "Field calibrated"
                if not fit_valid:
                    status = (perception.calib_fail or state.get("calibration_reason") or "Show the mapped floor tags across both ends.")
                    if status == "floor_reference_drift":
                        status = "A floor tag moved from its mapped position. Check tag placement and the camera mount."
                if needs_confirm:
                    status = "Take a snapshot and check the projected far corners against the field markings."
                evidence = {"source_session": self.session, "revision": state["calibration_revision"],
                            "seq": state["seq"], "captured_at": captured, "fit_valid": fit_valid,
                            "status": status, "ids": tags, "mode": setup["mode"],
                            "metrics": dict(perception.setup_metrics, reference_error_px=state["calib_err_px"]),
                            "lens": "calibrated lens" if perception.has_calibrated_intrinsics else "nominal lens model",
                            "goal_source": "surveyed physical posts" if goals else "centred mouth from configured dimensions",
                            "field": state["field"], "layout_guide": guide,
                            "ball_seen": visible["ball"], "robots_seen": visible["robots"],
                            "robot_status": {n: rb.get("tracking_detail", "") for n, rb in state["robots"].items()}}
                with self.lock:
                    match = self.director.update(state) if self.director else None
                    if match is not None:
                        state["fc_match"] = match
                        for pub in self.publishers:
                            pub.send(state)
                    self.state.update(status=status, live=live, frame=state, visible=visible, match=match,
                                      fit_valid=fit_valid, needs_confirm=needs_confirm, geometry=evidence)
                    self.projection = perception.cam, width, height
                if now >= next_preview:
                    next_preview = now + .2
                    # Pi ingress checks the nominal goal coordinates against its
                    # enrolled pitch. Surveyed posts travel in fc_match.goal_lines
                    # and are also the coordinates to paint for the operator.
                    preview_state = dict(state, goals=goals) if goals else state
                    view = perception.draw(frame, preview_state, banner=False)
                    candidates = []
                    if self.appearance:
                        from final_project.overhead_perception.fieldcam.appearance import detect_puppers
                        for candidate in detect_puppers(frame):
                            point = tuple(round(x) for x in candidate.center_px)
                            cv2.drawMarker(view, point, (255, 195, 60), cv2.MARKER_DIAMOND, 18, 2)
                            candidates.append({"pixel": list(point), "score": float(candidate.score)})
                    ok, jpg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 76])
                    if ok:
                        with self.lock:
                            self.picture = jpg.tobytes()
                            self.preview_meta = evidence
                            if setup["mode"] == "corner":
                                self.preview_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                            self.state["appearance"] = candidates
                if not live:
                    self.done.wait(1 / cfg["camera"]["fps"])
        except Exception as exc:
            with self.lock:
                self.state["status"] = str(exc) or type(exc).__name__
        finally:
            if source is not None:
                source.close()
            with self.lock:
                self.state.update(live=False, frame=None, visible={}, match=None)
