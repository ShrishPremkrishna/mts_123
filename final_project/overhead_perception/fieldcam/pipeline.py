"""Frame in -> shared field state out."""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import planner
from .ballistics import RollModel, RollTracker
from .camera_model import CameraModel, default_intrinsics, load_intrinsics, solve_camera_pose
from .config import resolve, set_field_size
from .detect import BallDetector, TagDetector
from .filters import CVFilter2D, YawFilter, wrap
from .intercept import RobotModel, plan_robot
from .relative import relative_geometry
from .states import BallMachine, RobotMachine
from .tagmeasure import survey_tags, tag_corners

CALIBRATION_DRIFT_LIMIT_PX = 8.0


def _quad_center(c: np.ndarray) -> np.ndarray:
    """Intersection of the diagonals: the true centre of a square under perspective."""
    p1, p2, p3, p4 = c[0], c[2], c[1], c[3]
    d1, d2 = p2 - p1, p4 - p3
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-9:
        return c.mean(axis=0)
    t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / denom
    return p1 + t * d1


class Perception:
    def __init__(self, cfg: dict, width: int, height: int):
        self.cfg = cfg
        self.size = (width, height)
        cam_cfg = cfg["camera"]
        intr = load_intrinsics(resolve(cam_cfg["intrinsics"]), width, height)
        self.has_calibrated_intrinsics = intr is not None
        self.K, self.dist = intr or default_intrinsics(width, height, cam_cfg["hfov_deg"])
        self.hfov_deg = None if intr else float(cam_cfg["hfov_deg"])   # the lens angle in use when it is only a guess
        self.fov_fit: dict | None = None    # set when calibration replaced that guess with a fitted angle

        self.cam: CameraModel | None = None
        self.roi_mask: np.ndarray | None = None
        self.ball_radius_px: float | None = None
        self.calib_err_px: float | None = None
        self.calib_mode: str | None = None  # 'tags' | 'survey' | 'clicks'
        self.floor_layout: dict = {}        # floor tag id -> field xy, for the bumped-camera check
        self.survey: dict | None = None     # set by calibrate_from_survey: reference tag + tag poses
        self.calib_fail: str | None = None  # why the last calibration attempt failed, in plain words
        self.ignored_tags: list = []        # floor-tag ids left out of the last corner calibration (not on the floor)
        self.calibration_valid = False
        self.calibration_revision = 0
        self.calibration_reason: str | None = None
        self.calibration_checked_at: float | None = None
        self._config_field = (cfg["field"]["length"], cfg["field"]["width"])  # as written in the config

        self.tag_det = TagDetector()
        b = cfg["ball"]
        self.ball_det = BallDetector(b["colors"])
        tr = cfg["tracking"]
        self.ball_f = CVFilter2D(tr["ball_accel_sigma"], tr["meas_sigma"])
        self.robot_f = {
            name: (CVFilter2D(tr["robot_accel_sigma"], tr["meas_sigma"]), YawFilter())
            for name in cfg["robots"]
        }
        # Where the ball is going (ballistics.py) and where each robot should meet it (intercept.py).
        self.roll_model, self.robot_model = RollModel.from_config(cfg), RobotModel.from_config(cfg)
        self.roll = RollTracker(self.roll_model)
        # What is going on, in words (states.py): one machine for the ball, one per robot.
        self.ball_machine = BallMachine()
        self.robot_machines = {name: RobotMachine(name, self.robot_model.kick_windup_s) for name in cfg["robots"]}
        self._plans: dict = {}              # last frame's plans, so routes and paws are not swapped for nothing
        self._frame_times: list = []        # recent capture times, for the measured frame rate
        self._ball_seen: list = []          # was the ball measured in each recent frame
        self.seq = 0
        self.debug: dict = {}  # raw detections from the last frame, for drawing

    # ---- calibration -----------------------------------------------------
    def calibrate(self, frame: np.ndarray, min_tags: int = 4) -> bool:
        """Locate the camera from the floor tags. Do this with the field EMPTY, then freeze."""
        tags = self.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        field = self.cfg["tags"]["field"]
        ids = [i for i in tags if i in field]
        if len(ids) < min_tags:
            return False
        cam = self._solve_camera([_quad_center(tags[i]) for i in ids], [field[i] for i in ids])
        return self._use_camera(cam, "tags", dict(field))

    def _solve_camera(self, image_pts, field_xy) -> CameraModel | None:
        """Camera pose from floor points, fitting the lens angle too when there is no lens calibration.

        Without an intrinsics file the lens is a pinhole guessed from camera.hfov_deg. A guess
        that is a few degrees out leaves the floor points several pixels from where the model
        puts them (measured on an iPhone looking down at an angle: 5.5 px at the configured
        69 degrees, 0.6 px at 66). So the angle is searched within 15 degrees of the configured
        one and replaced only when that at least halves the error: looking straight down, the
        angle cannot be told from the camera's height and the error barely changes, so the
        configured value stays.
        """
        cam = solve_camera_pose(self.K, self.dist, image_pts, field_xy)
        if self.has_calibrated_intrinsics or len(image_pts) < 6:
            return cam
        base = cam.reproj_err_px if cam is not None else float("inf")
        configured = float(self.cfg["camera"]["hfov_deg"])

        def at(hfov):
            K, _ = default_intrinsics(*self.size, hfov)
            c = solve_camera_pose(K, self.dist, image_pts, field_xy)
            return (c.reproj_err_px if c is not None else float("inf")), hfov, c, K

        best = min((at(h) for h in np.arange(configured - 15.0, configured + 15.01, 1.0)), key=lambda r: r[0])
        best = min((at(h) for h in np.arange(best[1] - 1.0, best[1] + 1.01, 0.1)), key=lambda r: r[0])
        err, hfov, fitted, K = best
        if fitted is None or not (err < 0.5 * base and base - err > 0.5):
            return cam
        self.K, self.hfov_deg = K, float(hfov)
        self.fov_fit = {"hfov_deg": float(hfov), "configured_deg": configured, "fit_px": float(err),
                        "fit_px_configured": None if cam is None else float(base)}
        return fitted

    def calibrate_from_survey(self, frame: np.ndarray, min_tags: int = 2, use_ids=None) -> bool:
        """Locate the camera from whatever floor tags are in view, with nothing measured.

        The lowest-numbered floor tag becomes the field centre: its right edge
        points at the east goal (+x), its top edge at +y. The other tags can lie
        anywhere on the floor; their positions are worked out from the image,
        using only the printed tag size (tags.floor_size). Works from 2 tags,
        better with 3+, spread apart. Robot tags are ignored.
        """
        floor = self._floor_tags(frame, use_ids)
        if len(floor) < min_tags:
            self.calib_fail = f"need at least {min_tags} floor tags completely in view; seeing {sorted(floor) or 'none'}"
            return False
        ref = min(floor)
        poses = self._survey(floor, ref)
        cam, layout = self._camera_from_layout(floor, poses)
        if not self._use_camera(cam, "survey", layout):
            self.calib_fail = "the tags in view do not fit a camera above the floor"
            return False
        self.survey = {"ref": ref, "tags": poses}
        return True

    def calibrate_from_corners(self, frame: np.ndarray, use_ids=None) -> bool:
        """Four floor tags on the field's corners define the field itself. Nothing is measured.

        tags.corner_ids says which tag is on which corner (default 0, 1, 2, 3
        going counter-clockwise seen from above, starting at the west goal's
        -y end). From those four the field's centre, direction and SIZE are
        worked out: +x runs from the 0-3 side to the 1-2 side, so the east goal
        is between tags 1 and 2. tags.corner_inset moves the true corner
        outward from the tag centre if the sheets sit inside the lines.

        Only the four corner tags locate the camera. Any other floor tag in
        view is checked against that: one lying flat on the floor is kept and
        watched for movement like the corners; one that is not (on a desk, on
        a chair, held in a hand) would bend the fit, so it is left out and
        listed in self.ignored_tags.
        """
        floor = self._floor_tags(frame, use_ids)
        roles = self.cfg["tags"]["corner_ids"]  # west_south, east_south, east_north, west_north
        missing = [f"tag {tid} ({name.replace('_', '-')})" for name, tid in roles.items() if tid not in floor]
        if missing:
            self.calib_fail = "corner tags not completely in view: " + ", ".join(missing)
            return False
        corners_only = {tid: floor[tid] for tid in roles.values()}
        poses = self._survey(corners_only, roles["west_south"])
        at = {name: np.array([poses[tid]["x"], poses[tid]["y"]]) for name, tid in roles.items()}
        ex = (at["east_south"] - at["west_south"]) + (at["east_north"] - at["west_north"])
        if np.linalg.norm(ex) < 1e-6:
            self.calib_fail = "the corner tags are on top of each other"
            return False
        ex = ex / np.linalg.norm(ex)                # +x: from the west side to the east side
        ey = np.array([-ex[1], ex[0]])              # +y: to the left of +x
        if ((at["west_north"] - at["west_south"]) + (at["east_north"] - at["east_south"])) @ ey <= 0:
            ids = [roles[k] for k in ("west_south", "east_south", "east_north", "west_north")]
            self.calib_fail = (f"the corner tags go round the field the wrong way. Seen from above, tags "
                               f"{ids[0]}, {ids[1]}, {ids[2]}, {ids[3]} must go counter-clockwise: swap tags {ids[1]} and {ids[3]}")
            return False
        origin = sum(at.values()) / 4
        to_field = lambda p: np.array([(p - origin) @ ex, (p - origin) @ ey])  # noqa: E731
        f = {name: to_field(p) for name, p in at.items()}
        len_tags = ((f["east_south"][0] - f["west_south"][0]) + (f["east_north"][0] - f["west_north"][0])) / 2
        wid_tags = ((f["west_north"][1] - f["west_south"][1]) + (f["east_north"][1] - f["east_south"][1])) / 2
        ideal = {"west_south": (-len_tags / 2, -wid_tags / 2), "east_south": (len_tags / 2, -wid_tags / 2),
                 "east_north": (len_tags / 2, wid_tags / 2), "west_north": (-len_tags / 2, wid_tags / 2)}
        off_rectangle = max(float(np.linalg.norm(f[k] - ideal[k])) for k in f)
        inset = self.cfg["tags"]["corner_inset"]
        length, width = len_tags + 2 * inset[0], wid_tags + 2 * inset[1]
        if length < 0.3 or width < 0.3:
            self.calib_fail = f"the corner tags are too close together to be a field ({length:.2f} x {width:.2f} m)"
            return False
        theta = math.atan2(ex[1], ex[0])
        field_poses = {}
        for tid, p in poses.items():
            xy = to_field(np.array([p["x"], p["y"]]))
            field_poses[tid] = {"x": float(xy[0]), "y": float(xy[1]), "yaw": wrap(p["yaw"] - theta)}
        cam, layout = self._camera_from_layout(corners_only, field_poses)
        if cam is None:
            self.calib_fail = "the corner tags do not fit a camera above the floor"
            return False
        # Other tags in view: on the floor, a tag's corners land on a square of the printed size.
        # Raised off the floor it looks bigger than that (a desk-height tag by about 40 %).
        size, ignored = self.cfg["tags"]["floor_size"], []
        for tid in sorted(set(floor) - set(corners_only)):
            c = cam.pixel_to_plane(floor[tid], 0.0)
            sides = [float(np.linalg.norm(c[k] - c[(k + 1) % 4])) for k in range(4)]
            diagonals = [float(np.linalg.norm(c[0] - c[2])), float(np.linalg.norm(c[1] - c[3]))]
            flat = (all(abs(side - size) < 0.12 * size for side in sides)
                    and abs(diagonals[0] - diagonals[1]) < 0.12 * size * math.sqrt(2))
            if flat:
                centre = c.mean(axis=0)
                layout[tid] = centre
                field_poses[tid] = {"x": float(centre[0]), "y": float(centre[1]), "yaw": 0.0}
            else:
                ignored.append(int(tid))
        if not self._use_camera(cam, "corners", layout):
            return False
        self.ignored_tags = ignored
        set_field_size(self.cfg, length, width)  # the field is now what the tags say it is
        self._set_field_roi()
        self.survey = {"ref": roles["west_south"], "tags": field_poses,
                       "field": {"length": length, "width": width, "off_rectangle": off_rectangle}}
        return True

    def _floor_tags(self, frame: np.ndarray, use_ids=None) -> dict:
        """Floor tags completely visible in this frame: {id: image corners}. Robot tags left out."""
        self.calib_fail = None
        tags = self.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        robot_ids = {rc["tag_id"] for rc in self.cfg["robots"].values()}
        return {i: c for i, c in tags.items() if i not in robot_ids and (use_ids is None or i in use_ids)}

    def _survey(self, floor: dict, ref: int) -> dict:
        """Where the floor tags are relative to tag `ref`, from the image and the printed size alone."""
        size = self.cfg["tags"]["floor_size"]
        # Survey on lens-corrected pixels; the camera pose solve takes the raw ones.
        flat = {i: cv2.undistortPoints(c.reshape(-1, 1, 2).astype(np.float64), self.K, self.dist, P=self.K)
                .reshape(4, 2) for i, c in floor.items()}
        poses, _ = survey_tags(flat, {i: size for i in floor}, ref)
        return poses

    def _camera_from_layout(self, floor: dict, poses: dict):
        """Camera pose from tags whose floor poses are known: (camera, {id: field xy})."""
        size, ids = self.cfg["tags"]["floor_size"], sorted(floor)
        cam = self._solve_camera(
            np.vstack([floor[i] for i in ids]),
            np.vstack([tag_corners((poses[i]["x"], poses[i]["y"], poses[i]["yaw"]), size) for i in ids]))
        return cam, {i: np.array([poses[i]["x"], poses[i]["y"]]) for i in ids}

    def calibrate_from_points(self, image_pts, field_xy) -> bool:
        """Locate the camera from >= 4 hand-picked floor points (no tags needed).

        image_pts: pixels in the full-resolution frame. field_xy: where those
        points are on the field, metres. Spread them wide and not all in a line.
        """
        if len(image_pts) < 4 or len(image_pts) != len(field_xy):
            return False
        return self._use_camera(solve_camera_pose(self.K, self.dist, image_pts, field_xy), "clicks", {})

    def _use_camera(self, cam: CameraModel | None, mode: str, floor_layout: dict) -> bool:
        if cam is None:
            return False
        if not math.isfinite(cam.reproj_err_px) or cam.reproj_err_px > CALIBRATION_DRIFT_LIMIT_PX:
            self.calib_fail = "camera calibration does not fit the reference points within 8 pixels"
            self.invalidate_calibration("calibration_fit_failed")
            return False
        if mode != "corners":  # every other mode uses the field size written in the config
            set_field_size(self.cfg, *self._config_field)
        self.cam = cam
        self.calib_mode, self.floor_layout = mode, floor_layout
        self.calib_err_px = cam.reproj_err_px
        self.calibration_valid = True
        self.calibration_revision += 1
        self.calibration_reason = None
        self.calibration_checked_at = None
        self.calib_fail = None

        # A new transform must not inherit positions/velocities from the previous one.
        tr = self.cfg["tracking"]
        self.ball_f = CVFilter2D(tr["ball_accel_sigma"], tr["meas_sigma"])
        self.robot_f = {
            name: (CVFilter2D(tr["robot_accel_sigma"], tr["meas_sigma"]), YawFilter())
            for name in self.cfg["robots"]
        }
        self.roll.reset()
        self._set_field_roi()

        r = self.cfg["ball"]["radius"]
        right = cam.R[0]  # camera x axis expressed in the field frame
        px = cam.project([[0, 0, r], list(right * r + np.array([0, 0, r]))])
        self.ball_radius_px = float(np.linalg.norm(px[1] - px[0]))
        return True

    def invalidate_calibration(self, reason: str) -> None:
        """Latch a hold while retaining the frozen transform for inspection.

        A later good frame does not clear this. Only an explicit successful
        calibration selects a new revision and clears the latch. Capture owners
        must call this on reconnect/source changes, even if floor tags are hidden.
        """
        self.calibration_valid = False
        self.calibration_reason = reason

    def _set_field_roi(self) -> None:
        # Only look for the ball on (and just around) the field.
        corners = np.array(self.cfg["field"]["corners"], float)
        grown = corners + 0.25 * np.sign(corners)
        poly = self.cam.project(np.hstack([grown, np.zeros((4, 1))])).astype(np.int32)
        self.roi_mask = np.zeros((self.size[1], self.size[0]), np.uint8)
        cv2.fillConvexPoly(self.roi_mask, poly, 255)

    def _ball_radius_at(self, u: float, v: float, r: float | None = None) -> float | None:
        """How many pixels a ball of radius r (default: the configured one) should span
        if its centre shows at pixel (u, v)."""
        r = self.cfg["ball"]["radius"] if r is None else r
        xy = self.cam.pixel_to_plane([[u, v]], r)[0]
        if not np.isfinite(xy).all():
            return None
        c = np.array([xy[0], xy[1], r])
        px = self.cam.project([c, c + self.cam.R[0] * r])
        size = float(np.linalg.norm(px[1] - px[0]))
        return size if 2.0 < size < 2000.0 else None

    def estimate_ball_radius(self, u: float, v: float, radius_px: float) -> float | None:
        """Real radius (m) of a ball on the floor that shows at (u, v) with this pixel radius.

        An ESTIMATE: it is only as good as the calibration and the lens model.
        A tape measure beats it.
        """
        if self.cam is None:
            return None
        r = self.cfg["ball"]["radius"]
        for _ in range(8):  # the centre height depends on r, so iterate to a fixed point
            px = self._ball_radius_at(u, v, r)
            if not px:
                return None
            r *= radius_px / px
        return float(r) if 0.005 < r < 0.5 else None

    # ---- per frame -------------------------------------------------------
    def process(self, frame: np.ndarray, t_capture: float, t_now: float | None = None) -> dict:
        """t_capture: when the photons hit the sensor. t_now: when this state will be used.

        Reported positions are extrapolated to t_now; age_s says how old the
        newest real measurement behind each one is. Control consumers must use
        calibration_valid and measured_this_frame; visible includes display coasting.
        """
        t_now = t_capture if t_now is None else t_now
        cfg = self.cfg
        self.seq += 1
        frame_size_changed = (frame.shape[1], frame.shape[0]) != self.size
        if frame_size_changed:
            self.invalidate_calibration("frame_size_changed")
        state = {
            "seq": self.seq, "stamp": t_now, "captured_at": t_capture, "frame_age_s": t_now - t_capture,
            "calibrated": self.calibration_valid, "calib_err_px": self.calib_err_px,
            "calibration_valid": self.calibration_valid, "calibration_revision": self.calibration_revision,
            "calibration_status": "uncalibrated" if self.cam is None else "invalidated",
            "calibration_reason": self.calibration_reason, "calibration_checked_at": self.calibration_checked_at,
            "ball": {"visible": False, "measured_this_frame": False, "measured_at": None},
            "robots": {n: {"visible": False, "measured_this_frame": False, "measured_at": None}
                       for n in cfg["robots"]},
            "goals": cfg["goals"], "shots": {}, "relative": {},
            # Where the ball is going, and where each robot should go because of it. None / empty
            # unless calibration is valid and the ball is being measured.
            "prediction": None, "intercepts": {}, "machines": None, "diagnostics": {},
            # Enough for a viewer to draw the scene without the config file.
            "field": {"length": cfg["field"]["length"], "width": cfg["field"]["width"],
                      "ball_radius": cfg["ball"]["radius"]},
            "calib_mode": self.calib_mode, "floor_tags": {},
        }
        self.debug = {"tags": {}, "ball_px": None}
        if frame_size_changed:
            # Intrinsics/ROI belong to the selected resolution. Do not project
            # this frame with them or let a later old-sized frame clear the hold.
            return state
        if self.cam is None:
            # Not calibrated: no positions, but still show what is being seen.
            self.debug["tags"] = self.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            self.debug["ball_px"] = self.ball_det.detect(frame)
            return state
        cam = self.cam
        lost = cfg["tracking"]["lost_after_s"]

        tags = self.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        self.debug["tags"] = tags

        # Calibration health: has the camera been bumped since we froze the pose?
        field = self.floor_layout
        seen = [i for i in tags if i in field]
        # Where each floor tag is on the field, and whether it is readable right now.
        state["floor_tags"] = {str(i): {"x": float(xy[0]), "y": float(xy[1]), "visible": i in tags}
                               for i, xy in field.items()}
        if seen:
            proj = cam.project([[*field[i], 0.0] for i in seen])
            obs = np.array([_quad_center(tags[i]) for i in seen])
            # Any moved reference invalidates the selected transform. A median
            # could conceal a displaced tag while most other tags remain fixed.
            self.calib_err_px = float(np.max(np.linalg.norm(proj - obs, axis=1)))
            self.calibration_checked_at = t_capture
            if not math.isfinite(self.calib_err_px) or self.calib_err_px > CALIBRATION_DRIFT_LIMIT_PX:
                self.invalidate_calibration("floor_reference_drift")
            state["calib_err_px"] = self.calib_err_px
        state.update(
            calibrated=self.calibration_valid, calibration_valid=self.calibration_valid,
            calibration_reason=self.calibration_reason, calibration_checked_at=self.calibration_checked_at,
            calibration_status=("invalidated" if not self.calibration_valid else
                                "verified" if seen else "occluded" if field else "unmonitored"),
        )

        # Robots: tag corners -> plane at tag height -> centre + heading.
        for name, rc in cfg["robots"].items():
            pos_f, yaw_f = self.robot_f[name]
            measured = False
            if rc["tag_id"] in tags:
                c = cam.pixel_to_plane(tags[rc["tag_id"]], rc["tag_height"])  # TL, TR, BR, BL
                fwd = (c[0] + c[1]) / 2 - (c[3] + c[2]) / 2
                yaw = wrap(math.atan2(fwd[1], fwd[0]) + rc["yaw_offset"])
                cy, sy = math.cos(yaw), math.sin(yaw)
                off = rc["tag_offset"]
                body = c.mean(axis=0) - np.array([cy * off[0] - sy * off[1], sy * off[0] + cy * off[1]])
                if pos_f.update(body, t_capture):
                    measured = yaw_f.update(yaw, t_capture)
            if pos_f.initialised and yaw_f.yaw is not None:
                x = pos_f.predict_to(t_now)
                state["robots"][name] = {
                    "visible": (t_capture - pos_f.t_meas) < lost,
                    "x": float(x[0]), "y": float(x[1]),
                    "yaw": wrap(yaw_f.yaw + yaw_f.rate * (t_now - yaw_f.t)),
                    "vx": float(x[2]), "vy": float(x[3]), "wz": float(yaw_f.rate),
                    "age_s": t_now - pos_f.t_meas,
                    "measured_this_frame": measured, "measured_at": pos_f.t_meas,
                }

        # Ball: orange blob -> plane through the ball's centre.
        det = self.ball_det.detect(frame, self.roi_mask, self.ball_radius_px, self._ball_radius_at)
        measured = False
        if det is not None:
            self.debug["ball_px"] = det
            xy = cam.pixel_to_plane([det[:2]], cfg["ball"]["radius"])[0]
            measured = self.ball_f.update(xy, t_capture)
            self.roll.update(xy, t_capture)
        if self.ball_f.initialised:
            x = self.ball_f.predict_to(t_now)
            state["ball"] = {
                "visible": (t_capture - self.ball_f.t_meas) < lost,
                "x": float(x[0]), "y": float(x[1]), "vx": float(x[2]), "vy": float(x[3]),
                "age_s": t_now - self.ball_f.t_meas,
                "measured_this_frame": measured, "measured_at": self.ball_f.t_meas,
            }

        # Shooting geometry per robot.
        ball = state["ball"]
        if self.calibration_valid and ball["visible"]:
            for name, rc in cfg["robots"].items():
                robot = state["robots"][name]
                others = [r for n, r in state["robots"].items() if n != name and r["visible"]]
                keeper_xy = (others[0]["x"], others[0]["y"]) if others else None
                target = planner.pick_target((ball["x"], ball["y"]), cfg["goals"][rc["attacks"]],
                                             cfg["shot"]["post_margin"], keeper_xy)
                shot = planner.shot_geometry((ball["x"], ball["y"]), target, cfg["shot"]["standoff"])
                if robot["visible"]:
                    shot["error"] = planner.shot_errors(robot, shot)
                    shot["ready"] = planner.shot_ready(robot, ball, shot, cfg["shot"])
                    shot["cmd"] = planner.approach_cmd(robot, ball, shot, cfg["approach"])
                state["shots"][name] = shot

        # The same scene from each robot's point of view (see relative.py for conventions).
        for name, rc in cfg["robots"].items():
            robot = state["robots"][name]
            if self.calibration_valid and robot["visible"]:
                state["relative"][name] = relative_geometry(
                    robot, ball if ball["visible"] else None, cfg["goals"][rc["attacks"]])

        # Where the ball is going (a roll model on a velocity fitted over the last few frames),
        # then, per robot, where to line up for a strike and whether its own goal needs a block.
        f = cfg["field"]
        own_goal = {name: next(g for side, g in cfg["goals"].items() if side != rc["attacks"])
                    for name, rc in cfg["robots"].items()}
        problem = None
        try:
            # The tracker itself drops a ball it has not seen for too long, so a ball hidden for a
            # moment (behind a robot) keeps its prediction.
            predicted = (self.roll.predict(t_now, f["length"], f["width"], f["goal_width"], cfg["ball"]["radius"])
                         if self.calibration_valid else None)
            if predicted is not None:
                state["prediction"], traj = predicted
                for name, rc in cfg["robots"].items():
                    robot = state["robots"][name]
                    if robot["visible"]:
                        state["intercepts"][name] = plan_robot(robot, traj, cfg["goals"][rc["attacks"]], own_goal[name],
                                                               f["length"], f["width"], self.robot_model,
                                                               area=state["prediction"]["area"],
                                                               previous=self._plans.get(name))
            self._plans = state["intercepts"]
            ball_now = self.ball_machine.update(state["prediction"], t_now)
            state["machines"] = {"ball": ball_now, "robots": {
                name: machine.update(t_now, self.calibration_valid, state["robots"][name], ball_now,
                                     state["intercepts"].get(name),
                                     {n: p for n, p in state["intercepts"].items() if n != name}, own_goal[name])
                for name, machine in self.robot_machines.items()}}
        except Exception as exc:  # noqa: BLE001  prediction and planning are extras: a fault there must not stop tracking
            problem = f"{type(exc).__name__}: {exc}"
            state["prediction"], state["intercepts"], state["machines"] = None, {}, None
        state["diagnostics"] = self._diagnostics(t_capture, tags, measured, problem)
        return state

    def _diagnostics(self, t_capture: float, tags: dict, ball_measured: bool, problem: str | None) -> dict:
        """Health figures for the dashboard: how fast frames come, what the camera fit looks like, what is in view."""
        self._frame_times = [t for t in self._frame_times if t_capture - t < 2.0 and t < t_capture] + [t_capture]
        self._ball_seen = (self._ball_seen + [bool(ball_measured)])[-40:]
        span = self._frame_times[-1] - self._frame_times[0]
        robot_ids = {rc["tag_id"]: name for name, rc in self.cfg["robots"].items()}
        return {
            "fps": (len(self._frame_times) - 1) / span if span > 0 else None,
            "lens_deg": self.hfov_deg, "lens_from": ("a lens calibration file" if self.has_calibrated_intrinsics else
                                                    "fitted from the tags" if self.fov_fit else "the config's guess"),
            "camera_height": float(self.cam.center[2]) if self.cam is not None else None,
            "fit_px": float(self.cam.reproj_err_px) if self.cam is not None else None,
            "drift_px": self.calib_err_px, "drift_limit_px": CALIBRATION_DRIFT_LIMIT_PX,
            "floor_tags_seen": sorted(int(i) for i in tags if i not in robot_ids),
            "robot_tags_seen": sorted(robot_ids[i] for i in tags if i in robot_ids),
            "ball_measured": bool(ball_measured), "ball_noise": float(self.roll.noise),
            "ball_seen_rate": sum(self._ball_seen) / len(self._ball_seen),
            "ignored_tags": list(self.ignored_tags),
            "problem": problem,
        }

    # ---- debug drawing ---------------------------------------------------
    def draw(self, frame: np.ndarray, state: dict, banner: bool = True) -> np.ndarray:
        """banner=False: the caller draws its own calibration messages."""
        out = frame.copy()
        for tid, c in self.debug.get("tags", {}).items():
            cv2.polylines(out, [c.astype(np.int32)], True, (255, 200, 0), 2)
            cv2.putText(out, str(tid), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        if self.cam is None:
            if banner:
                cv2.putText(out, "NOT CALIBRATED   c: 4+ measured floor tags   t: any 2+ floor tags, nothing measured   m: click corners",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            if self.debug.get("ball_px"):
                u, v, r = self.debug["ball_px"]
                cv2.circle(out, (int(u), int(v)), int(r), (0, 255, 0), 3)
                cv2.putText(out, "ball found (no position until calibrated)", (int(u - r), int(v - r) - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            return out
        P = lambda x, y, z=0.0: tuple(int(v) for v in self.cam.project([[x, y, z]])[0])  # noqa: E731

        corners = self.cfg["field"]["corners"]
        for a, b in zip(corners, corners[1:] + corners[:1]):
            cv2.line(out, P(*a), P(*b), (255, 255, 255), 1)
        for g in state["goals"].values():
            cv2.line(out, P(g["x"], g["y_min"]), P(g["x"], g["y_max"]), (0, 255, 255), 4)

        ball = state["ball"]
        if ball.get("visible"):
            r = self.cfg["ball"]["radius"]
            cv2.circle(out, P(ball["x"], ball["y"], r), int(self.ball_radius_px or 8) + 4, (0, 255, 0), 2)
            bx, by = P(ball["x"], ball["y"], r)
            cv2.putText(out, f"ball ({ball['x'] * 100:+.0f}, {ball['y'] * 100:+.0f}) cm", (bx + 20, by - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            pred = state.get("prediction") or {}
            if pred.get("moving") and len(pred.get("path") or []) > 1:
                # Where it is going: the predicted roll, and a ring where it ends.
                path = np.int32([P(x, y, r) for _, x, y in pred["path"]])
                cv2.polylines(out, [path], False, (190, 105, 255), 3, cv2.LINE_AA)
                cv2.circle(out, P(*pred["end"], r), int(self.ball_radius_px or 8) + 4, (190, 105, 255), 2)
                area = pred.get("area")
                if area:   # the stopping area, drawn on the floor
                    c, s_ = math.cos(area["heading"]), math.sin(area["heading"])
                    ring = [P(area["x"] + c * area["along"] * math.cos(a) - s_ * area["across"] * math.sin(a),
                              area["y"] + s_ * area["along"] * math.cos(a) + c * area["across"] * math.sin(a))
                            for a in np.linspace(0, 2 * math.pi, 40)]
                    cv2.polylines(out, [np.int32(ring)], True, (190, 105, 255), 2, cv2.LINE_AA)
                for _, x, y in pred.get("trail") or []:   # what earlier frames of this roll predicted
                    cv2.circle(out, P(x, y, r), 3, (255, 255, 255), -1, cv2.LINE_AA)
            elif "moving" not in pred:
                cv2.arrowedLine(out, P(ball["x"], ball["y"], r),
                                P(ball["x"] + ball["vx"] * 0.5, ball["y"] + ball["vy"] * 0.5, r), (0, 255, 0), 2)
        for name, rb in state["robots"].items():
            if not rb.get("visible"):
                continue
            h = self.cfg["robots"][name]["tag_height"]
            tip = (rb["x"] + 0.2 * math.cos(rb["yaw"]), rb["y"] + 0.2 * math.sin(rb["yaw"]))
            cv2.arrowedLine(out, P(rb["x"], rb["y"], h), P(*tip, h), (255, 0, 255), 3, tipLength=0.3)
            label = f"{name} ({rb['x'] * 100:+.0f}, {rb['y'] * 100:+.0f}) cm, {math.degrees(rb['yaw']):+.0f} deg"
            cv2.putText(out, label, P(rb["x"], rb["y"], h), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
            rel = state.get("relative", {}).get(name, {}).get("ball")
            if rel and ball.get("visible"):
                # The ball as this robot sees it: how far ahead, how far to its left, how much to turn.
                a, b = P(rb["x"], rb["y"], h), P(ball["x"], ball["y"], self.cfg["ball"]["radius"])
                cv2.line(out, a, b, (255, 255, 0), 2, cv2.LINE_AA)
                side = "left" if rel["bearing"] >= 0 else "right"
                text = (f"ball {rel['distance'] * 100:.0f} cm, turn {abs(math.degrees(rel['bearing'])):.0f} deg {side}"
                        f"  ({rel['x'] * 100:+.0f} ahead, {rel['y'] * 100:+.0f} left)")
                cv2.putText(out, text, (a[0], a[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            shot = state["shots"].get(name)
            if shot:
                sx, sy, syaw = shot["staging"]
                col = (0, 255, 0) if shot.get("ready") else (0, 165, 255)
                cv2.line(out, P(sx, sy), P(*shot["target"]), col, 1)
                cv2.circle(out, P(sx, sy), 8, col, 2)
        hud = f"calib err {state['calib_err_px']:.1f}px  frame age {state['frame_age_s'] * 1000:.0f}ms"
        if self.calib_mode == "corners":
            fs = self.survey["field"]
            hud += f"  |  field from corner tags: {fs['length']:.2f} x {fs['width']:.2f} m"
        if self.calib_mode == "survey":
            hud += f"  |  tag {self.survey['ref']} is the field centre, its right edge points at the east goal"
        cv2.putText(out, hud, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return out
