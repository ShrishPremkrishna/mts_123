"""Frame in -> shared field state out."""
from __future__ import annotations

import math
from itertools import combinations

import cv2
import numpy as np

from . import planner
from .camera_model import CameraModel, default_intrinsics, load_intrinsics, solve_camera_pose
from .config import resolve, set_field_size
from .corner_anchor import FIELD_TOLERANCE_M, anchor_geometry, corner_fit
from .detect import BallDetector, TagDetector
from .filters import CVFilter2D, YawFilter, wrap
from .relative import relative_geometry
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

        self.cam: CameraModel | None = None
        self.roi_mask: np.ndarray | None = None
        self.ball_scale = 1.
        self.ball_radius_px: float | None = None
        self.calib_err_px: float | None = None
        self.calib_mode: str | None = None  # 'tags' | 'survey' | 'clicks'
        self.floor_layout: dict = {}        # floor tag id -> field xy, for the bumped-camera check
        self.survey: dict | None = None     # set by calibrate_from_survey: reference tag + tag poses
        self.far_corner = None
        self.setup_metrics = {}
        self.calib_fail: str | None = None  # why the last calibration attempt failed, in plain words
        self.calibration_valid = False
        self.calibration_revision = 0
        self.calibration_reason: str | None = None
        self.calibration_checked_at: float | None = None
        self._config_field = (cfg["field"]["length"], cfg["field"]["width"])  # as written in the config

        mapped = cfg["tags"]["field"] if cfg.get("field_setup", {}).get("mode") == "layout" else ()
        self.tag_det = TagDetector(mapped)
        b = cfg["ball"]
        self.ball_det = BallDetector(b["colors"])
        tr = cfg["tracking"]
        self.ball_f = CVFilter2D(tr["ball_accel_sigma"], tr["meas_sigma"])
        self.robot_f = {
            name: (CVFilter2D(tr["robot_accel_sigma"], tr["meas_sigma"]), YawFilter())
            for name in cfg["robots"]
        }
        self.seq = 0
        self.debug: dict = {}  # raw detections from the last frame, for drawing

    # ---- calibration -----------------------------------------------------
    def calibrate(self, frame: np.ndarray, min_tags: int = 4) -> bool:
        """Locate the camera from the floor tags. Do this with the field EMPTY, then freeze."""
        tags = self.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        field = self.cfg["tags"]["field"]
        ids = [i for i in tags if i in field]
        self.calib_fail, self.setup_metrics = None, {}
        setup = self.cfg.get("field_setup", {})
        duplicates = self.tag_det.duplicates & set(field)
        if duplicates:
            self.calib_fail = f"Duplicate floor tag IDs {sorted(duplicates)}. Remove the extra copies."
            return False
        if setup.get("mode") == "corner":
            anchor_id = setup["tag_id"]
            if anchor_id not in tags:
                self.calib_fail = f"Show the whole black square and white border of corner tag {anchor_id}."
                return False
            anchor = anchor_geometry(setup)
            cam, self.calib_fail, self.setup_metrics = self._anchor_fit(frame, tags[anchor_id], anchor)
            return cam is not None and self._use_camera(cam, "corner_anchor", {anchor_id: anchor.mean(axis=0)})
        if len(ids) < min_tags:
            self.calib_fail = f"Found floor IDs {sorted(ids) or 'none'}. Uncover at least {min_tags} mapped tags across both ends."
            return False
        xy = np.array([field[i] for i in ids])
        if setup and not self._spans_field(xy, require_quad=True):
            self.calib_fail = "Visible floor tags are clustered. Uncover tags on both sidelines and both end lines."
            return False
        observed = np.array([_quad_center(tags[i]) for i in ids])
        cam = solve_camera_pose(self.K, self.dist, observed, xy)
        if cam is None:
            self.calib_fail = "The floor IDs do not fit an overhead camera. Check their placement and camera focus."
            return False
        errors = np.linalg.norm(cam.project(np.column_stack((xy, np.zeros(len(xy))))) - observed, axis=1)
        if errors.max() > CALIBRATION_DRIFT_LIMIT_PX:
            worst = ids[int(np.argmax(errors))]
            self.calib_fail = f"Floor tag {worst} is {errors.max():.1f} pixels from its mapped position. Check the tag IDs and placement."
            return False
        if setup and len(ids) >= 5:
            held_out = []
            for index, tag_id in enumerate(ids):
                others = np.arange(len(ids)) != index
                if not self._spans_field(xy[others], require_quad=True):
                    continue
                prediction = solve_camera_pose(self.K, self.dist, observed[others], xy[others])
                if prediction is None:
                    continue
                projected = prediction.project([[*xy[index], 0]])[0]
                held_out.append((float(np.linalg.norm(projected-observed[index])), tag_id))
            if held_out:
                error, tag_id = max(held_out)
                self.setup_metrics["held_out_max_px"] = round(error, 2)
                if error > CALIBRATION_DRIFT_LIMIT_PX:
                    self.calib_fail = f"Tag {tag_id} is {error:.1f} pixels from the position predicted by the other IDs. Check that floor label."
                    return False
        return self._use_camera(cam, "tags", dict(field))

    def _anchor_fit(self, frame, observed, anchor):
        corners = np.asarray(self.cfg["field"]["corners"])
        ref = None
        if self.far_corner is not None:
            uv = self.far_corner.locate(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if uv is None:
                return None, "The opposite corner mark moved or is hidden. Take a new snapshot and mark it again.", {}
            opposite = corners[np.argmax(np.linalg.norm(corners - anchor.mean(axis=0), axis=1))]
            ref = opposite, uv
        return corner_fit(self.K, self.dist, observed, anchor, corners, self.size, ref)

    def _spans_field(self, xy, require_quad=False):
        if len(xy) < 3:
            return False
        length, width = self._config_field
        if require_quad:
            # Three collinear references plus a single opposite point can
            # enclose area yet leave an unconstrained projective degree of freedom.
            def triangle_area(a, b, c):
                v, w = b - a, c - a
                return abs(v[0]*w[1] - v[1]*w[0]) / 2
            if not any(min(triangle_area(*triple) for triple in combinations(quad, 3)) > .025 * length * width
                       for quad in combinations(xy, 4)):
                return False
        return (np.all(np.ptp(xy, axis=0) >= np.array([length, width]) * .70)
                and cv2.contourArea(cv2.convexHull(np.float32(xy))) >= .38 * length * width)

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
        is between tags 1 and 2. Any other floor tags in view help locate the
        camera but do not affect the field. tags.corner_inset moves the true
        corner outward from the tag centre if the sheets sit inside the lines.
        """
        floor = self._floor_tags(frame, use_ids)
        roles = self.cfg["tags"]["corner_ids"]  # west_south, east_south, east_north, west_north
        missing = [f"tag {tid} ({name.replace('_', '-')})" for name, tid in roles.items() if tid not in floor]
        if missing:
            self.calib_fail = "corner tags not completely in view: " + ", ".join(missing)
            return False
        poses = self._survey(floor, roles["west_south"])
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
        cam, layout = self._camera_from_layout(floor, field_poses)
        if cam is None:
            self.calib_fail = "the corner tags do not fit a camera above the floor"
            return False
        if not self._use_camera(cam, "corners", layout):
            return False
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
        cam = solve_camera_pose(
            self.K, self.dist, np.vstack([floor[i] for i in ids]),
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
        self._set_field_roi()
        if self.cfg.get("robot_tracking") == "colors":
            from .patch_plane import PatchPlane
            f = self.cfg["field"]
            heights = {rc["tag_height"] for rc in self.cfg["robots"].values()}
            self.patch_planes = {h: PatchPlane(cam, h, f["length"], f["width"], self.size) for h in heights}

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
        # A large pink ball retains its outline at 896 px; keep native pixels
        # for the small fiducials that benefit from the higher resolution.
        self.ball_scale = min(1., 896 / self.size[0]) if self.cfg.get("field_setup") else 1.
        self.ball_roi = self.roi_mask if self.ball_scale == 1 else cv2.resize(
            self.roi_mask, None, fx=self.ball_scale, fy=self.ball_scale, interpolation=cv2.INTER_NEAREST)

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
    def process(self, frame: np.ndarray, t_capture: float, t_now: float | None = None, *, match_only=False) -> dict:
        """t_capture: when the photons hit the sensor. t_now: when this state will be used.

        Reported positions are extrapolated to t_now; yaw_at_capture preserves
        the measured heading separately from the display yaw. age_s says how old
        the newest real measurement behind each one is. Control consumers must use
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
            # Enough for a viewer to draw the scene without the config file.
            "field": {"length": cfg["field"]["length"], "width": cfg["field"]["width"],
                      "ball_radius": cfg["ball"]["radius"], "goal_width": cfg["field"]["goal_width"]},
            "calib_mode": self.calib_mode, "floor_tags": {},
            "setup_metrics": self.setup_metrics, "tracking": cfg.get("robot_tracking", "tags"),
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

        # The dashboard admits current distributed references, or a current check
        # of the single square and its extrapolation across the field.
        setup = cfg.get("field_setup")
        if setup and self.calibration_valid:
            reason = None
            if self.tag_det.duplicates & set(field):
                reason = "Duplicate floor ID. Remove the duplicate tag."
            elif self.calib_mode == "corner_anchor":
                anchor_id = setup["tag_id"]
                if anchor_id not in tags:
                    reason = f"Corner tag {anchor_id} is hidden. Uncover it and recheck the field."
                else:
                    field_xy = np.asarray(cfg["field"]["corners"])
                    measured_cam, reason, _ = self._anchor_fit(frame, tags[anchor_id], anchor_geometry(setup))
                    if measured_cam is not None:
                        reobserved = measured_cam.pixel_to_plane(cam.project(np.column_stack((field_xy, np.zeros(4)))), 0)
                        if np.linalg.norm(reobserved - field_xy, axis=1).max() > FIELD_TOLERANCE_M:
                            reason = "The single-tag field moved. Check the mount and confirm a new snapshot."
                if reason:
                    self.invalidate_calibration(reason)
            elif not self._spans_field(np.array([field[i] for i in seen])):
                reason = "Uncover mapped tags across both ends and sidelines to verify the camera."
            if reason:
                state.update(calibration_valid=False, calibrated=False, calibration_reason=reason,
                             calibration_status="unverified")

        color_poses = {}
        if state["tracking"] == "colors":
            self.debug["team_colors"] = {}
            for h, plane in self.patch_planes.items():
                for team, pose in plane.detect(frame).items():
                    if cfg["robots"][team]["tag_height"] != h:
                        continue
                    center = plane.to_xy(pose.center_px)
                    front = plane.to_xy(pose.front_px) if pose.front_px is not None else None
                    color_poses[team] = (center, front)
                    self.debug["team_colors"][team] = (plane.image_pixels(center)[0], front is not None)
        # Both color patches sit on the same standing body-height plane.
        for name, rc in cfg["robots"].items():
            pos_f, yaw_f = self.robot_f[name]
            measured = False
            body = yaw = None
            if state["tracking"] == "colors":
                pose = color_poses.get(name)
                detail = "No unique team patch"
                if pose is not None:
                    detail = "Team visible; signed heading needs a unique nose patch"
                    body, front = pose
                    if front is not None:
                        direction = front - body
                        if np.isfinite(direction).all() and .06 <= np.linalg.norm(direction) <= .126:
                            yaw = math.atan2(direction[1], direction[0])
                            detail = "Team and nose measured"
                state["robots"][name]["tracking_detail"] = detail
            elif rc["tag_id"] in tags:
                c = cam.pixel_to_plane(tags[rc["tag_id"]], rc["tag_height"])  # TL, TR, BR, BL
                fwd = (c[0] + c[1]) / 2 - (c[3] + c[2]) / 2
                yaw = wrap(math.atan2(fwd[1], fwd[0]) + rc["yaw_offset"])
                cy, sy = math.cos(yaw), math.sin(yaw)
                off = rc["tag_offset"]
                body = c.mean(axis=0) - np.array([cy * off[0] - sy * off[1], sy * off[0] + cy * off[1]])
            if yaw is not None and pos_f.update(body, t_capture):
                measured = yaw_f.update(yaw, t_capture)
            if pos_f.initialised and yaw_f.yaw is not None:
                x = pos_f.predict_to(t_now)
                state["robots"][name] = {
                    **state["robots"][name],
                    "visible": (t_capture - pos_f.t_meas) < lost,
                    "x": float(x[0]), "y": float(x[1]),
                    # A measured pair gives signed heading at capture. An abrupt
                    # reacquisition must never extrapolate the nose to the far side.
                    "yaw": yaw if measured and state["tracking"] == "colors"
                           else wrap(yaw_f.yaw + yaw_f.rate * (t_now - yaw_f.t)),
                    "yaw_at_capture": (wrap(yaw) if state["tracking"] == "colors"
                                       else wrap(yaw_f.yaw)) if measured else None,
                    "vx": float(x[2]), "vy": float(x[3]), "wz": float(yaw_f.rate),
                    "age_s": t_now - pos_f.t_meas,
                    "measured_this_frame": measured, "measured_at": pos_f.t_meas,
                }

        # Ball: orange blob -> plane through the ball's centre.
        if self.ball_scale < 1:
            scale = self.ball_scale
            small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            def small_radius(x, y):
                radius = self._ball_radius_at(x/scale, y/scale)
                return radius * scale if radius is not None else None
            det = self.ball_det.detect(small, self.ball_roi, self.ball_radius_px*scale, small_radius)
            if det is not None:
                det = tuple(v/scale for v in det)
        else:
            det = self.ball_det.detect(frame, self.roi_mask, self.ball_radius_px, self._ball_radius_at)
        measured = False
        if det is not None:
            self.debug["ball_px"] = det
            xy = cam.pixel_to_plane([det[:2]], cfg["ball"]["radius"])[0]
            measured = self.ball_f.update(xy, t_capture)
        if self.ball_f.initialised:
            x = self.ball_f.predict_to(t_now)
            state["ball"] = {
                "visible": (t_capture - self.ball_f.t_meas) < lost,
                "x": float(x[0]), "y": float(x[1]), "vx": float(x[2]), "vy": float(x[3]),
                "age_s": t_now - self.ball_f.t_meas,
                "measured_this_frame": measured, "measured_at": self.ball_f.t_meas,
            }

        if match_only:
            # The match director and walking executor own strategy and shot alignment.
            return state
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
        return state

    # ---- debug drawing ---------------------------------------------------
    def draw(self, frame: np.ndarray, state: dict, banner: bool = True) -> np.ndarray:
        """banner=False: the caller draws its own calibration messages."""
        out = frame.copy()
        for tid, c in self.debug.get("tags", {}).items():
            cv2.polylines(out, [c.astype(np.int32)], True, (255, 200, 0), 2)
            cv2.putText(out, str(tid), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        for team, (point, signed) in self.debug.get("team_colors", {}).items():
            center = tuple(round(float(i)) for i in point)
            color = (250, 140, 42) if team == "striker" else (68, 66, 245)
            cv2.circle(out, center, 7, color, 2)
            if not signed:
                cv2.putText(out, "nose?", (center[0]+8, center[1]), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1)
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
        if self.calib_mode == "corner_anchor":
            # The operator compares the *physical* floor intersections to these
            # metric tolerance rings. Their ellipse shape includes perspective.
            theta = np.linspace(0, 2 * math.pi, 36, endpoint=False)
            offsets = FIELD_TOLERANCE_M * np.column_stack((np.cos(theta), np.sin(theta)))
            for x, y in corners:
                ring = np.asarray([x, y]) + offsets
                pts = self.cam.project(np.column_stack((ring, np.zeros(len(ring)))))
                cv2.polylines(out, [np.int32(np.round(pts))], True, (0, 190, 255), 2, cv2.LINE_AA)
                side = ("E" if x > 0 else "W") + ("N" if y > 0 else "S")
                cv2.putText(out, side, P(x, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 190, 255), 1)
        for g in state["goals"].values():
            cv2.line(out, P(g["x"], g["y_min"]), P(g["x"], g["y_max"]), (0, 255, 255), 4)

        ball = state["ball"]
        if ball.get("visible"):
            r = self.cfg["ball"]["radius"]
            cv2.circle(out, P(ball["x"], ball["y"], r), int(self.ball_radius_px or 8) + 4, (0, 255, 0), 2)
            bx, by = P(ball["x"], ball["y"], r)
            cv2.putText(out, f"ball ({ball['x'] * 100:+.0f}, {ball['y'] * 100:+.0f}) cm", (bx + 20, by - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
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
