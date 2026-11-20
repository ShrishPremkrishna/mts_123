"""Own the overhead device once and publish one measured field frame to the selected match."""
from __future__ import annotations

import threading
import time
import uuid


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

    def observe_kick(self, event):
        with self.lock:
            if self.director:
                self.director.observe_kick(event)

    def survey(self, data):
        import math
        with self.lock:
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
            if simulation:
                from final_project.overhead_perception.scripts.run_perception import SimSource
                source = SimSource(cfg, (1280, 800))
            else:
                profile = self.profile
                cfg["camera"].update(profile.get("camera", {}))
                f = profile["field"]
                cfg["field"]["goal_width"], cfg["ball"]["radius"] = f["goal_width"], f["ball_radius"]
                set_field_size(cfg, f["length"], f["width"])
                cfg["tags"]["field"] = {int(i): np.array(p, float) for i, p in profile["tags"].items()}
                cfg["tags"]["corner_ids"] = profile["corner_ids"]
                cfg["tags"]["floor_size"] = profile["floor_size"]
                c = cfg["camera"]
                device = int(self.source) if self.source.isdigit() else self.source
                source = FrameGrabber(device, c["width"], c["height"], c["fps"], wait=2)
            live = not simulation and not source.is_file
            width = min(self.profile.get("processing_width", 896), source.width)
            height = round(source.height * width / source.width)
            perception = Perception(cfg, width, height)
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
                if (frame.shape[1], frame.shape[0]) != (source.width, source.height):
                    raise ValueError("Camera resolution changed. Reconnect the camera.")
                if width != source.width:
                    frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
                now = time.monotonic()
                with self.lock:
                    may_calibrate = self.director is None
                if may_calibrate and not perception.calibration_valid and now >= next_cal:
                    next_cal = now + .4
                    perception.calibrate(frame)
                captured = receipt - cfg["camera"]["latency_s"] if live else receipt
                if not live and not simulation:
                    captured = source.file_time_s(previous_video, None)
                    previous_video = captured
                state = perception.process(frame, captured, time.time() if live else captured, match_only=True)
                tags = sorted(perception.debug["tags"])
                visible = {"tags": tags, "ball": state["ball"]["measured_this_frame"],
                           "robots": {n: v["measured_this_frame"] for n, v in state["robots"].items()}}
                status = "Field calibrated" if state["calibration_valid"] else "Looking for four known field tags"
                if not tags:
                    status = "No tags visible. Check camera focus and framing."
                if state.get("calibration_reason"):
                    status = state["calibration_reason"]
                with self.lock:
                    match = self.director.update(state) if self.director else None
                    if match is not None:
                        state["fc_match"] = match
                        for pub in self.publishers:
                            pub.send(state)
                    self.state.update(status=status, live=live, frame=state, visible=visible, match=match)
                    self.projection = perception.cam, width, height
                if now >= next_preview:
                    next_preview = now + .2
                    view = perception.draw(frame, state, banner=False)
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
