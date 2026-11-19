"""Frame in -> shared field state out."""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import planner
from .camera_model import CameraModel, default_intrinsics, load_intrinsics, solve_camera_pose
from .config import resolve
from .detect import BallDetector, TagDetector
from .filters import CVFilter2D, YawFilter, wrap
from .relative import relative_geometry
from .tagmeasure import survey_tags, tag_corners


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
        self.ball_radius_px: float | None = None
        self.calib_err_px: float | None = None
        self.calib_mode: str | None = None  # 'tags' | 'survey' | 'clicks'
        self.floor_layout: dict = {}        # floor tag id -> field xy, for the bumped-camera check
        self.survey: dict | None = None     # set by calibrate_from_survey: reference tag + tag poses

        self.tag_det = TagDetector()
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
        if len(ids) < min_tags:
            return False
        cam = solve_camera_pose(self.K, self.dist,
                                [_quad_center(tags[i]) for i in ids], [field[i] for i in ids])
        return self._use_camera(cam, "tags", dict(field))

    def calibrate_from_survey(self, frame: np.ndarray, min_tags: int = 2, use_ids=None) -> bool:
        """Locate the camera from whatever floor tags are in view, with nothing measured.

        The lowest-numbered floor tag becomes the field centre: its right edge
        points at the east goal (+x), its top edge at +y. The other tags can lie
        anywhere on the floor; their positions are worked out from the image,
        using only the printed tag size (tags.floor_size). Works from 2 tags,
        better with 3+, spread apart. Robot tags are ignored.
        """
        tags = self.tag_det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        robot_ids = {rc["tag_id"] for rc in self.cfg["robots"].values()}
        floor = {i: c for i, c in tags.items()
                 if i not in robot_ids and (use_ids is None or i in use_ids)}
        if len(floor) < min_tags:
            return False
        size = self.cfg["tags"]["floor_size"]
        # Survey on lens-corrected pixels; the pose solve below takes the raw ones.
        flat = {i: cv2.undistortPoints(c.reshape(-1, 1, 2).astype(np.float64), self.K, self.dist, P=self.K)
                .reshape(4, 2) for i, c in floor.items()}
        ref = min(floor)
        poses, _ = survey_tags(flat, {i: size for i in floor}, ref)
        ids = sorted(floor)
        cam = solve_camera_pose(
            self.K, self.dist, np.vstack([floor[i] for i in ids]),
            np.vstack([tag_corners((poses[i]["x"], poses[i]["y"], poses[i]["yaw"]), size) for i in ids]))
        layout = {i: np.array([poses[i]["x"], poses[i]["y"]]) for i in ids}
        if not self._use_camera(cam, "survey", layout):
            return False
        self.survey = {"ref": ref, "tags": poses}
        return True

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
        self.cam = cam
        self.calib_mode, self.floor_layout = mode, floor_layout
        self.calib_err_px = cam.reproj_err_px

        # Only look for the ball on (and just around) the field.
        corners = np.array(self.cfg["field"]["corners"], float)
        grown = corners + 0.25 * np.sign(corners)
        poly = cam.project(np.hstack([grown, np.zeros((4, 1))])).astype(np.int32)
        self.roi_mask = np.zeros((self.size[1], self.size[0]), np.uint8)
        cv2.fillConvexPoly(self.roi_mask, poly, 255)

        r = self.cfg["ball"]["radius"]
        right = cam.R[0]  # camera x axis expressed in the field frame
        px = cam.project([[0, 0, r], list(right * r + np.array([0, 0, r]))])
        self.ball_radius_px = float(np.linalg.norm(px[1] - px[0]))
        return True

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
        newest real measurement behind each one is.
        """
        t_now = t_capture if t_now is None else t_now
        cfg = self.cfg
        self.seq += 1
        state = {
            "seq": self.seq, "stamp": t_now, "frame_age_s": t_now - t_capture,
            "calibrated": self.cam is not None, "calib_err_px": self.calib_err_px,
            "ball": {"visible": False}, "robots": {n: {"visible": False} for n in cfg["robots"]},
            "goals": cfg["goals"], "shots": {}, "relative": {},
            # Enough for a viewer to draw the scene without the config file.
            "field": {"length": cfg["field"]["length"], "width": cfg["field"]["width"],
                      "ball_radius": cfg["ball"]["radius"]},
            "calib_mode": self.calib_mode, "floor_tags": {},
        }
        self.debug = {"tags": {}, "ball_px": None}
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
            self.calib_err_px = float(np.median(np.linalg.norm(proj - obs, axis=1)))
            state["calib_err_px"] = self.calib_err_px

        # Robots: tag corners -> plane at tag height -> centre + heading.
        for name, rc in cfg["robots"].items():
            pos_f, yaw_f = self.robot_f[name]
            if rc["tag_id"] in tags:
                c = cam.pixel_to_plane(tags[rc["tag_id"]], rc["tag_height"])  # TL, TR, BR, BL
                fwd = (c[0] + c[1]) / 2 - (c[3] + c[2]) / 2
                yaw = wrap(math.atan2(fwd[1], fwd[0]) + rc["yaw_offset"])
                cy, sy = math.cos(yaw), math.sin(yaw)
                off = rc["tag_offset"]
                body = c.mean(axis=0) - np.array([cy * off[0] - sy * off[1], sy * off[0] + cy * off[1]])
                pos_f.update(body, t_capture)
                yaw_f.update(yaw, t_capture)
            if pos_f.initialised and yaw_f.yaw is not None:
                x = pos_f.predict_to(t_now)
                state["robots"][name] = {
                    "visible": (t_capture - pos_f.t_meas) < lost,
                    "x": float(x[0]), "y": float(x[1]),
                    "yaw": wrap(yaw_f.yaw + yaw_f.rate * (t_now - yaw_f.t)),
                    "vx": float(x[2]), "vy": float(x[3]), "wz": float(yaw_f.rate),
                    "age_s": t_now - pos_f.t_meas,
                }

        # Ball: orange blob -> plane through the ball's centre.
        det = self.ball_det.detect(frame, self.roi_mask, self.ball_radius_px, self._ball_radius_at)
        if det is not None:
            self.debug["ball_px"] = det
            xy = cam.pixel_to_plane([det[:2]], cfg["ball"]["radius"])[0]
            self.ball_f.update(xy, t_capture)
        if self.ball_f.initialised:
            x = self.ball_f.predict_to(t_now)
            state["ball"] = {
                "visible": (t_capture - self.ball_f.t_meas) < lost,
                "x": float(x[0]), "y": float(x[1]), "vx": float(x[2]), "vy": float(x[3]),
                "age_s": t_now - self.ball_f.t_meas,
            }

        # Shooting geometry per robot.
        ball = state["ball"]
        if ball["visible"]:
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
            if robot["visible"]:
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
        if self.calib_mode == "survey":
            hud += f"  |  tag {self.survey['ref']} is the field centre, its right edge points at the east goal"
        cv2.putText(out, hud, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return out
