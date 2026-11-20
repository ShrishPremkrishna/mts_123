#!/usr/bin/env python3
"""Main loop: camera (or simulator) -> field state -> UDP + debug window.

  python scripts/run_perception.py                 # live camera from config
  python scripts/run_perception.py --source 1      # another camera index
  python scripts/run_perception.py --source sim    # no hardware: simulated arena, striker walks to the shot
  python scripts/run_perception.py --source clip.mov

Keys: f = FIELD FROM CORNER TAGS: tags 0, 1, 2, 3 on the four corners of the field define it,
          position, direction and size, with nothing measured (--corners does it at start).
          Seen from above they go counter-clockwise; the east goal (+x) is between tags 1 and 2
      c = (re)calibrate from floor tags at the measured positions in the config (needs 4+)
      t = calibrate from whatever floor tags are in view, nothing measured (needs 2+):
          the lowest-numbered tag becomes the field centre, its right edge points at the east goal
      m = calibrate without tags: click the floor points listed in the config
          (default: the four field corners), in the order the window asks
      q = quit

Every pixel has a floor position once calibrated:
      move the mouse   = position of the floor spot under the cursor
      left click       = pin that spot; consecutive pins show the distance between them
      right click or x = clear the pins
      g = show / hide a measuring grid drawn on the floor
      p = save the position of every pixel to out/pixel_map.npy

Dashboard (one window: camera feed on the left, top-down field view on the right, readouts below):
      d = switch between that and the camera feed alone
      z = zoom the field view between the whole field and "fit to what is visible"
      (--dashboard starts with it; scripts/dashboard.py shows the field view from another terminal or machine)

Ball colour:
      b = put the cursor on the ball and press b: the ball colour is set from the
          pixels under the cursor, and the lines to paste into the config are printed
"""
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam import clickcal, colorpick  # noqa: E402
from fieldcam.appearance import AppearanceTracker, detect_puppers  # noqa: E402
from fieldcam.capture import FrameGrabber, is_camera_name, is_live, phone_on_cable  # noqa: E402
from fieldcam.camera_model import default_intrinsics  # noqa: E402
from fieldcam.config import SIM_CONFIG, load_config, resolve  # noqa: E402
from fieldcam.filters import wrap  # noqa: E402
from fieldcam.net import StatePublisher  # noqa: E402
from fieldcam.detect import BallDetector  # noqa: E402
from fieldcam.pipeline import Perception  # noqa: E402
from fieldcam.probe import Probe, floor_map  # noqa: E402
from fieldcam.topdown import render_combined  # noqa: E402


class SimSource:
    """Simulated arena where the striker obeys the published approach command."""

    def __init__(self, cfg, size):
        from fieldcam.sim import SimArena
        K, _ = default_intrinsics(*size, cfg["camera"]["hfov_deg"])
        self.arena = SimArena(cfg, K, size=size)
        self.width, self.height = size
        self.robots = {"striker": [-1.0, -0.5, 0.3], "keeper": [1.35, 0.1, math.pi]}
        self.ball = np.array([0.3, 0.2])
        self.t, self.n, self.dt = 0.0, 0, 1 / 30
        self.empty = True  # first frame: empty field for calibration

    def step(self, state):
        cmd = (state or {}).get("shots", {}).get("striker", {}).get("cmd")
        if cmd and not state["shots"]["striker"].get("ready"):
            x, y, yaw = self.robots["striker"]
            c, s = math.cos(yaw), math.sin(yaw)
            self.robots["striker"] = [x + (c * cmd["vx"] - s * cmd["vy"]) * self.dt,
                                      y + (s * cmd["vx"] + c * cmd["vy"]) * self.dt,
                                      wrap(yaw + cmd["wz"] * self.dt)]

    def read(self, last_n=-1):
        self.n += 1
        self.t += self.dt
        if self.empty:
            self.empty = False
            return self.arena.render({}, None), self.t, self.n
        return self.arena.render({k: tuple(v) for k, v in self.robots.items()}, self.ball), self.t, self.n

    def close(self):
        pass


WINDOW = "pupper fc perception"
OPEN_CAMERAS = []  # released on exit, however the program ends
CAMERA_HELP = "Wireless phone: lock it, keep it still and near the Mac, Wi-Fi and Bluetooth on."
CLICKS_FILE = "out/manual_calib.json"


RULE = "=" * 72
DONE_BANNER_S = 4.0  # how long the green "done" banner stays on screen


def box(title: str, lines=()):
    """A message that is hard to miss in a scrolling terminal."""
    print("\n" + RULE + f"\n  {title}\n" + "".join(f"  {ln}\n" for ln in lines) + RULE, flush=True)


def floor_tags_in_view(perc) -> list:
    """Floor tags that are completely visible in the last frame (robot tags left out)."""
    robot_ids = {rc["tag_id"] for rc in perc.cfg["robots"].values()}
    return sorted(i for i in perc.debug.get("tags", {}) if i not in robot_ids)


def corner_text(perc) -> str:
    r = perc.cfg["tags"]["corner_ids"]
    return (f"tag {r['west_south']} and tag {r['west_north']} on the west goal line, "
            f"tag {r['east_south']} and tag {r['east_north']} on the east goal line")


def needed(perc, mode: str):
    """(how many floor tags calibration needs, which of the ones in view count). mode: corners | survey | measured."""
    seen = floor_tags_in_view(perc)
    if mode == "corners":
        return 4, [i for i in seen if i in perc.cfg["tags"]["corner_ids"].values()]
    if mode == "survey":
        return 2, seen
    return 4, [i for i in seen if i in perc.cfg["tags"]["field"]]


def announce_needed(perc, mode: str):
    if mode == "corners":
        r = perc.cfg["tags"]["corner_ids"]
        box("CALIBRATION NEEDED: FIELD FROM CORNER TAGS", [
            "Put one floor tag on each corner of the field, tag centre on the corner:",
            f"  seen from above, counter-clockwise: tag {r['west_south']}, tag {r['east_south']}, "
            f"tag {r['east_north']}, tag {r['west_north']}",
            f"  the east goal (+x) is between tags {r['east_south']} and {r['east_north']}, "
            f"the west goal between tags {r['west_north']} and {r['west_south']}",
            "All four must be COMPLETELY in view: whole black square and white border, nothing on top.",
            "It calibrates by itself the moment it sees them. The field's size is taken from the tags.",
        ])
    elif mode == "survey":
        box("CALIBRATION NEEDED", [
            "Put at least 2 floor tags where the camera sees them COMPLETELY:",
            "the whole black square and the white border around it, nothing on top.",
            "It calibrates by itself the moment it sees them. 3 or more, spread apart, is better.",
        ])
    else:
        box("CALIBRATION NEEDED", [
            "Waiting for at least 4 floor tags at the measured positions in the config, completely in view.",
            "No measured layout yet?  Press t in the window: any 2+ floor tags, nothing measured.",
            "No tags at all?          Press m in the window and click the field corners.",
        ])


def announce_done(perc):
    cam, mode = perc.cam, perc.calib_mode
    how = {"tags": f"{len(perc.floor_layout)} measured floor tags in the config",
           "survey": f"{len(perc.survey['tags']) if perc.survey else 0} floor tags, nothing measured",
           "corners": f"the four corner tags ({len(perc.survey['tags']) if perc.survey else 0} floor tags in view), nothing measured",
           "clicks": "clicked points"}.get(mode, str(mode))
    lines = [f"from: {how}",
             f"camera is {cam.center[2]:.2f} m above the floor; fit error {cam.reproj_err_px:.2f} px (under 2 is good)"]
    if mode == "corners":
        fs, r = perc.survey["field"], perc.cfg["tags"]["corner_ids"]
        lines.append(f"THE FIELD IS {fs['length']:.2f} m long (goal to goal) x {fs['width']:.2f} m wide, measured from the tags")
        lines.append(f"east goal (+x) between tags {r['east_south']} and {r['east_north']}, "
                     f"west goal between tags {r['west_north']} and {r['west_south']}; centre is the middle of the four")
        if fs["off_rectangle"] > 0.03:
            lines.append(f"NOTE: the four tags are {fs['off_rectangle'] * 100:.0f} cm away from a true rectangle. "
                         "Square the corners up, then press f again.")
        else:
            lines.append(f"the four tags form a rectangle to within {fs['off_rectangle'] * 100:.1f} cm")
        if fs["length"] < fs["width"]:
            lines.append("NOTE: the goals are on the LONG sides. If that is not intended, the tags are turned a quarter: "
                         "move each tag one corner round.")
    if mode == "survey":
        sv = perc.survey
        lines.append(f"tag {sv['ref']} is the field centre: its right edge points at the east goal (+x), top edge at +y")
        lines += [f"tag {tid} is at x {p['x'] * 100:+.1f} cm, y {p['y'] * 100:+.1f} cm from tag {sv['ref']}"
                  for tid, p in sorted(sv["tags"].items()) if tid != sv["ref"]]
        if len(sv["tags"]) < 3:
            lines.append("NOTE: only 2 tags, so expect several cm of error. A third tag, spread apart, helps a lot.")
    if cam.reproj_err_px > 3.0:
        lines.append("NOTE: the fit error is high. Check the tag size / positions in the config, then recalibrate.")
    lines.append("Positions are live now. If the camera or the tags move, recalibrate (f, t, c or m).")
    box("CALIBRATION DONE", lines)


def survey_calibrate(perc, frame, quiet: bool = False) -> bool:
    """Calibrate from the floor tags in view without any measured positions."""
    if not perc.calibrate_from_survey(frame):
        if not quiet:
            seen = floor_tags_in_view(perc) if perc.cam is None else sorted(perc.debug.get("tags", {}))
            box("CALIBRATION FAILED", [f"Need at least 2 floor tags completely in view; seeing {seen or 'none'}.",
                                       "Robot tags do not count. The previous calibration, if any, is still in use."])
        return False
    return True


def corners_calibrate(perc, frame, quiet: bool = False) -> bool:
    """Calibrate with the four corner tags defining the field."""
    if not perc.calibrate_from_corners(frame):
        if not quiet:
            box("CALIBRATION FAILED", [perc.calib_fail or "could not calibrate from the corner tags",
                                       "Expected: " + corner_text(perc) + ".",
                                       "The previous calibration, if any, is still in use."])
        return False
    return True


def draw_banner(img, title, lines, colour):
    """A filled strip across the top of the picture."""
    h = 70 + 44 * len(lines)
    strip = img[:h].copy()
    cv2.rectangle(strip, (0, 0), (img.shape[1], h), (0, 0, 0), -1)
    img[:h] = cv2.addWeighted(strip, 0.92, img[:h], 0.08, 0)
    cv2.putText(img, title, (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.5, colour, 4, cv2.LINE_AA)
    for k, text in enumerate(lines):
        cv2.putText(img, text, (24, 100 + 44 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 2, cv2.LINE_AA)


def status_line(state) -> str:
    """One line a person can read: where things are and how far apart."""
    if not state["calibrated"]:
        return "not calibrated"
    parts, ball = [], state["ball"]
    parts.append(f"ball ({ball['x'] * 100:+.0f}, {ball['y'] * 100:+.0f}) cm" if ball["visible"] else "ball not seen")
    for name, r in state["robots"].items():
        if not r["visible"]:
            parts.append(f"{name} not seen")
            continue
        text = f"{name} ({r['x'] * 100:+.0f}, {r['y'] * 100:+.0f}) cm facing {math.degrees(r['yaw']):+.0f} deg"
        rel = state.get("relative", {}).get(name, {}).get("ball")
        if rel:
            side = "left" if rel["bearing"] >= 0 else "right"
            text += (f"; ball {rel['x'] * 100:+.0f} cm ahead, {rel['y'] * 100:+.0f} cm left "
                     f"= {rel['distance'] * 100:.0f} cm away, turn {abs(math.degrees(rel['bearing'])):.0f} deg {side}")
        parts.append(text)
    return " | ".join(parts)


def pick_ball_colour(perc, frame, cursor):
    """Set the ball colour from the pixels under the cursor and say what to put in the config."""
    if cursor is None:
        box("BALL COLOUR NOT SET", ["Move the mouse onto the ball in the window first, then press b."])
        return
    try:
        ranges = colorpick.suggest_ranges(frame, *cursor)
    except ValueError as err:
        box("BALL COLOUR NOT SET", [str(err)])
        return
    perc.ball_det = BallDetector(ranges)
    perc.cfg["ball"]["colors"] = ranges
    lines = ["In use now, until the program exits. To keep it, paste under `ball: colors:` in config/field.yaml:"]
    lines += colorpick.as_yaml(ranges)
    found = perc.ball_det.detect(frame, perc.roi_mask)  # no size expectation: see what is really there
    if found is None:
        lines.append("With this colour no ball is found in the current frame. Try pointing at a brighter part of it.")
    else:
        est = perc.estimate_ball_radius(*found)
        if est is not None:
            lines.append(f"The ball looks about {est * 100:.1f} cm in radius; the config says "
                         f"{perc.cfg['ball']['radius'] * 100:.1f} cm. This is an estimate from the camera: measure it to be sure.")
        else:
            lines.append("Ball found in the picture. (No size estimate until the camera is calibrated.)")
    box("BALL COLOUR SET", lines)


def click_calibrate(perc, frame, points, scale) -> bool:
    """Freeze this frame, collect one click per point, and locate the camera from them."""
    box("CLICK CALIBRATION", [f"Click these {len(points)} floor points in the window, in this order:"]
        + [f"{k + 1}. {p['name']}" for k, p in enumerate(points)] + ["u = undo the last click, Esc = cancel"])
    pts = clickcal.collect_clicks(WINDOW, frame, points, scale)
    if pts is None:
        box("CALIBRATION CANCELLED", ["The previous calibration, if any, is still in use."])
        return False
    if not perc.calibrate_from_points(pts, [p["xy"] for p in points]):
        box("CALIBRATION FAILED", ["The clicked points do not fit a camera above the floor.",
                                   "Check the click order and the positions in the config, then press m again."])
        return False
    clickcal.save_clicks(resolve(CLICKS_FILE), pts, points, perc.size)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--source", default=None, help="camera index, video path, or 'sim'")
    ap.add_argument("--replay-fps", type=float, help="assume this constant frame rate for a recorded file in place of its presentation timestamps")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run forever)")
    ap.add_argument("--snapshot", default=None, help="write the last annotated frame to this image file")
    ap.add_argument("--print", action="store_true", help="print each state as JSON")
    ap.add_argument("--reuse-clicks", action="store_true",
                    help="start from the last click calibration (only valid if the camera has not moved)")
    ap.add_argument("--corners", action="store_true",
                    help="calibrate at start from tags on the four field corners, which define the field (same as key f)")
    ap.add_argument("--survey", action="store_true",
                    help="calibrate at start from the floor tags in view, nothing measured (same as key t)")
    ap.add_argument("--dashboard", action="store_true",
                    help="show the top-down field view and readouts beside the camera feed (same as key d)")
    ap.add_argument("--appearance-preview", action="store_true",
                    help="overlay anonymous body centers from camera pixels for comparison")
    ap.add_argument("--appearance-length-px", type=float,
                    help="expected Pupper length in the processed image; defaults to 7.5 percent of width")
    ap.add_argument("--dashboard-width", type=int, default=1400, help="width of that combined window in pixels")
    ap.add_argument("--display-width", type=int, default=1280, help="debug window width in pixels")
    ap.add_argument("--processing-width", type=int, help="downsample each captured frame to at most this width before calibration and detection")
    args = ap.parse_args()
    if args.processing_width is not None and args.processing_width < 1:
        ap.error("--processing-width must be positive")
    if args.appearance_length_px is not None and not 0 < args.appearance_length_px < math.inf:
        ap.error("--appearance-length-px must be positive and finite")

    cfg = load_config(args.config or (SIM_CONFIG if args.source == "sim" else None))
    cam_cfg = cfg["camera"]
    source = args.source if args.source is not None else cam_cfg["source"]
    if isinstance(source, str) and source.isdigit():
        source = int(source)
    sim = source == "sim"
    live = not sim and is_live(source)
    replay = not sim and not live
    if args.replay_fps is not None and (not replay or not 0 < args.replay_fps < math.inf):
        ap.error("--replay-fps requires a video file and a finite positive frame rate")
    previous_file_time = None

    if sim:
        cfg["camera"]["intrinsics"] = "config/_none_.yaml"  # the simulated lens is an ideal pinhole
        src = SimSource(cfg, (cam_cfg["width"], cam_cfg["height"]))
    else:
        # A live camera may be a wireless phone that is not there yet: wait for it, for as long as it takes.
        src = FrameGrabber(source, cam_cfg["width"], cam_cfg["height"], cam_cfg["fps"], wait=None if live else 0)
        OPEN_CAMERAS.append(src)
    print(f"source {source!r}: {src.width}x{src.height}")
    if live and is_camera_name(source):
        cable = phone_on_cable()
        print("phone link: " + {True: "USB cable", False: "WIRELESS (no phone on USB): a cable is steadier", None: "unknown"}[cable])

    width = min(args.processing_width or src.width, src.width)
    height = max(1, round(src.height * width / src.width))
    if width < src.width:
        print(f"processing at {width}x{height}; capture timestamps are preserved")
    perc = Perception(cfg, width, height)
    if not perc.has_calibrated_intrinsics and not sim:
        print("WARNING: no intrinsics file; using a pinhole guess from hfov_deg. "
              "Run scripts/calibrate_intrinsics.py for real accuracy.")
    pub = StatePublisher(cfg["net"]["host"], cfg["net"]["port"], live=live)
    import atexit
    atexit.register(pub.close)
    print(json.dumps({"event": "perception_source", "source_id": pub.source_id,
                      "source_session": pub.source_session, "live": live}))
    latency = cam_cfg["latency_s"] if live else 0.0
    points = cfg["manual_points"]
    scale = min(1.0, args.display_width / width)
    if args.reuse_clicks:
        saved = clickcal.load_clicks(resolve(CLICKS_FILE), points, perc.size)
        if saved is not None and perc.calibrate_from_points(saved, [p["xy"] for p in points]):
            print("using the saved click calibration")
        else:
            print("no usable saved clicks (none saved, or the resolution or point list changed)")

    n, state, vis, t_fps, fps = -1, None, None, time.time(), 0.0
    count, t_status = 0, 0.0
    reconnects = getattr(src, "reconnects", 0)
    probe, mouse_ready = Probe(scale), False
    appearance = AppearanceTracker() if args.appearance_preview else None
    dash_on, dash_zoom = args.dashboard, "field"

    def show(picture, stale_s=0.0):
        """Put the camera picture in the window, alone or with the field view and readouts."""
        if dash_on:
            canvas, k, bounds = render_combined(picture, state, args.dashboard_width, dash_zoom, stale_s)
            probe.scale, probe.bounds = k, bounds
        else:
            canvas = cv2.resize(picture, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1.0 else picture
            probe.scale, probe.bounds = scale, None
        cv2.imshow(WINDOW, canvas)
    announced_cam, asked, t_done, moved = None, False, 0.0, False  # what has been said about calibration so far
    feed_lost = False
    auto = "corners" if args.corners else "survey" if args.survey else "measured"  # how to calibrate at start
    while True:
        frame, t_rx, n = src.read(n)
        new_reconnects = getattr(src, "reconnects", 0)
        if new_reconnects != reconnects:
            reconnects = new_reconnects
            perc.invalidate_calibration("camera reconnected; verify setup and recalibrate")
            if appearance is not None:
                appearance = AppearanceTracker()
        if frame is None:
            if not live:
                break
            # No new frame. If that lasts, the feed is lost: say so, publish nothing (consumers
            # then see their data go stale, which is the truth), and wait for it to come back.
            lost = src.lost_for
            if lost and not feed_lost:
                feed_lost = True
                perc.invalidate_calibration("camera feed lost; verify setup and recalibrate")
                box("CAMERA FEED LOST", ["No frames are arriving. Nothing is being published while it is down.",
                                         "Trying to reconnect by itself. " + CAMERA_HELP])
            if lost and not args.no_gui:
                dead = (vis * 0.35).astype(np.uint8) if vis is not None else np.zeros((height, width, 3), np.uint8)
                draw_banner(dead, "CAMERA FEED LOST", [
                    f"no frames for {lost:.0f} s: reconnecting by itself",
                    "wireless phone: lock it, keep it still and near the Mac    q = quit"], (0, 0, 255))
                show(dead, stale_s=lost)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            continue
        if feed_lost:
            feed_lost = False
            box("CAMERA FEED BACK", ["Frames are arriving again. Motion remains withheld after the interruption;",
                                     "verify the camera and field, then explicitly recalibrate."])
        if width < src.width:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        if perc.cam is None and auto != "measured":
            if count % 10 == 0:  # --corners / --survey: never fall back to the measured layout in the config
                (corners_calibrate if auto == "corners" else survey_calibrate)(perc, frame, quiet=True)
        elif perc.cam is None:
            perc.calibrate(frame)
        # Live: capture happened `latency` before we received the frame, and "now" is the wall clock.
        t_capture = t_rx - latency
        if replay:
            t_capture = src.file_time_s(previous_file_time, args.replay_fps)
            previous_file_time = t_capture
        state = perc.process(frame, t_capture, time.time() if live else t_capture)
        if replay:
            state["replay_clock"] = ("video_presentation" if args.replay_fps is None else "assumed_constant_fps")
            if args.replay_fps is not None:
                state["replay_fps"] = args.replay_fps
        pub.send(state)
        if sim:
            src.step(state)
        bodies = []
        if appearance is not None and (not args.no_gui or args.print or args.snapshot):
            length_px = args.appearance_length_px or frame.shape[1] * .075
            bodies = appearance.update(detect_puppers(frame, length_px), t_capture, length_px)
        if args.print:
            diagnostic = state if appearance is None else {**state, "appearance_preview": [vars(b) for b in bodies]}
            print(json.dumps(diagnostic))

        # ---- say clearly where calibration stands ----
        if perc.cam is not announced_cam:      # a new calibration just happened (auto or by key)
            announced_cam, t_done, moved = perc.cam, time.time(), False
            announce_done(perc)
        elif perc.cam is None and not asked:   # tried on the first frame and could not
            asked = True
            announce_needed(perc, auto)
        err = state.get("calib_err_px") or 0.0
        invalid_calibration = not state.get("calibration_valid", False)
        if perc.cam is not None and invalid_calibration != moved:
            moved = invalid_calibration
            if moved:
                box("CALIBRATION WITHHELD", [state.get("calibration_reason", "calibration needs verification"),
                                             "Verify the field and camera, then explicitly recalibrate (f / c / m)."])
            else:
                print("new calibration accepted; bind its revision before robot navigation", flush=True)
        if not args.print and (live or perc.cam is None) and time.time() - t_status > 1.0:
            t_status = time.time()
            if perc.cam is None:
                need, have = needed(perc, auto)
                what = "corner tags" if auto == "corners" else "floor tags"
                print(f"CALIBRATING... {what} completely in view: {have or 'none'}  ({len(have)} of {need} needed)",
                      flush=True)
            elif live:
                print(status_line(state), flush=True)

        count += 1
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_fps, 1e-6)
        t_fps = now
        if not args.no_gui or (args.snapshot and args.frames and count >= args.frames):
            vis = perc.draw(frame, state, banner=False)
            for body in bodies:
                x, y = map(round, body.center_px)
                color = (255, 210, 30)
                cv2.drawMarker(vis, (x, y), color, cv2.MARKER_CROSS, 16, 2)
                label = "body overlap" if body.ambiguous else f"body {body.score:.2f}"
                cv2.putText(vis, label, (x + 9, y - 5), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1)
            probe.draw(vis, perc.cam)
            if perc.cam is None:
                need, have = needed(perc, auto)
                keys = ("it starts by itself" if auto != "measured" else "f = tags on the 4 corners define the field"
                        "    t = any 2+ tags, nothing measured") + "    m = click the field corners"
                what = "corner tags (one on each field corner)" if auto == "corners" else "floor tags"
                draw_banner(vis, "CALIBRATING...", [
                    f"{what} completely in view: {len(have)} of {need} needed"
                    + (f"   (ids {', '.join(map(str, have))})" if have else ""),
                    "a tag counts only if its whole black square and white border are visible",
                    keys], (0, 200, 255))
            elif moved:
                draw_banner(vis, "CALIBRATION WITHHELD", [
                    state.get("calibration_reason", "verify calibration before motion"),
                    "recalibrate: f = corner tags, t = any tags, c = measured layout, m = click"], (0, 0, 255))
            elif time.time() - t_done < DONE_BANNER_S:
                draw_banner(vis, "CALIBRATION DONE", [
                    f"camera {perc.cam.center[2]:.2f} m above the floor, fit error {perc.cam.reproj_err_px:.1f} px",
                    "positions are live. Recalibrate with t if the camera or tags move"], (0, 230, 0))
            else:
                cv2.putText(vis, f"{fps:.0f} fps", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if not args.no_gui:
            show(vis)
            if not mouse_ready:  # the window has to exist before it can take a mouse handler
                cv2.setMouseCallback(WINDOW, probe.on_mouse)
                mouse_ready = True
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("g"):
                probe.grid = not probe.grid
            if key == ord("d"):
                dash_on = not dash_on
            if key == ord("z"):
                dash_zoom = "fit" if dash_zoom == "field" else "field"
            if key == ord("x"):
                probe.pins.clear()
            if key == ord("b"):
                pick_ball_colour(perc, frame, probe.cursor)
            if key == ord("p") and perc.cam is not None:
                path = resolve("out/pixel_map.npy")
                path.parent.mkdir(parents=True, exist_ok=True)
                np.save(path, floor_map(perc.cam, width, height))
                print(f"saved {path.name}: array [row, column] -> (x, y) in metres on the floor, "
                      "NaN where a pixel does not show floor. Valid until the camera moves.")
            if key == ord("c") and not perc.calibrate(frame):
                _need, have = needed(perc, "measured") if perc.cam is None else (4, sorted(perc.debug.get("tags", {})))
                box("CALIBRATION FAILED", [f"Need at least 4 of the measured floor tags completely in view; seeing {have or 'none'}.",
                                           "The previous calibration, if any, is still in use."])
            if key == ord("f"):
                corners_calibrate(perc, frame)
            if key == ord("t"):
                survey_calibrate(perc, frame)
            if key == ord("m"):
                click_calibrate(perc, frame, perc.cfg["manual_points"], scale)  # current field size, e.g. after f
                mouse_ready = False  # the click mode borrowed the mouse; take it back
        if args.frames and count >= args.frames:
            break

    if args.snapshot and vis is not None:
        cv2.imwrite(args.snapshot, vis)
        print("wrote", args.snapshot)
    if state:
        s = state["shots"].get("striker", {})
        print(f"{count} frames, ~{fps:.0f} fps. striker ready={s.get('ready')} error={s.get('error')}")
    pub.close()


def _stop(*_args):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _stop)  # a plain kill also releases the camera
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        # Always let go of the camera. A wireless phone left held by a dead program
        # can stay unavailable to the next run.
        for grabber in OPEN_CAMERAS:
            grabber.close()
        cv2.destroyAllWindows()
