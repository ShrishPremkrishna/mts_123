"""A top-down "dashboard" picture of the field, drawn from the published state alone.

It needs no camera and no config file: everything comes from one state dict
(the same JSON that goes out over UDP), so it can run in the perception
process or on any machine that receives the stream.

Layout: the field on the left (+x to the right, +y up, like the gameplay
viewer), a text panel on the right. Only what perception currently reports is
drawn solid; anything not seen is listed as "not seen", never left on screen
as if it were still there.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

PANEL_W = 400
ROBOT_LEN, ROBOT_WID = 0.34, 0.20  # drawn footprint, metres (for looks only)
COLOURS = {"striker": (255, 80, 255), "keeper": (0, 170, 255)}
GRASS, LINE, PINK = (62, 110, 52), (235, 235, 235), (190, 105, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _field_dims(state: dict):
    f = state.get("field") or {}
    goals = state.get("goals") or {}
    length = f.get("length") or 2 * abs(next(iter(goals.values()), {"x": 1.83})["x"])
    return float(length), float(f.get("width") or length * 2 / 3), float(f.get("ball_radius") or 0.08)


def _bounds(state: dict, zoom: str):
    """World rectangle to show: the whole field, or just what is currently visible."""
    length, width, _ = _field_dims(state)
    whole = (-length / 2 - 0.5, length / 2 + 0.5, -width / 2 - 0.35, width / 2 + 0.35)
    if zoom != "fit":
        return whole
    pts = [(t["x"], t["y"]) for t in (state.get("floor_tags") or {}).values()]
    pts += [(r["x"], r["y"]) for r in (state.get("robots") or {}).values() if r.get("visible")]
    if (state.get("ball") or {}).get("visible"):
        pts.append((state["ball"]["x"], state["ball"]["y"]))
    if not pts:
        return whole
    xs, ys = zip(*pts)
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half = max(max(xs) - min(xs), max(ys) - min(ys), 0.6) / 2 + 0.3
    return (cx - half, cx + half, cy - half, cy + half)


def view_mapping(state: dict, size, zoom: str = "field"):
    """How the field view lays the field out in a picture of this size.
    Returns (to_px, to_world): field metres -> pixel, and pixel -> field metres."""
    w, h = size
    x0, x1, y0, y1 = _bounds(state, zoom)
    scale = min((w - 40) / (x1 - x0), (h - 40) / (y1 - y0))
    ox = 20 + ((w - 40) - scale * (x1 - x0)) / 2
    oy = 20 + ((h - 40) - scale * (y1 - y0)) / 2
    to_px = lambda x, y: (int(round(ox + (x - x0) * scale)), int(round(oy + (y1 - y) * scale)))  # noqa: E731
    to_world = lambda u, v: (x0 + (u - ox) / scale, y1 - (v - oy) / scale)  # noqa: E731
    to_px.scale = scale
    return to_px, to_world


def _dashed(img, a, b, colour, thick=1, dash=10):
    a, b = np.array(a, float), np.array(b, float)
    n = max(int(np.linalg.norm(b - a) // dash), 1)
    for k in range(0, n, 2):
        p, q = a + (b - a) * k / n, a + (b - a) * min(k + 1, n) / n
        cv2.line(img, tuple(np.int32(p)), tuple(np.int32(q)), colour, thick, cv2.LINE_AA)


def _text(img, text, org, colour=(235, 235, 235), scale=0.55, thick=1):
    cv2.putText(img, text, (int(org[0]), int(org[1])), FONT, scale, colour, thick, cv2.LINE_AA)


def render_field(state: dict | None, size=(880, 800), zoom: str = "field", stale_s: float = 0.0) -> np.ndarray:
    """Just the top-down drawing. state: a perception state (None = nothing received yet).
    zoom: "field" or "fit". stale_s: how long ago this state arrived; it greys out when old."""
    w, h = size
    img = np.full((h, w, 3), (28, 28, 28), np.uint8)
    view_w = w
    if state is None:
        _text(img, "NO DATA", (max(view_w // 2 - 110, 8), h // 2), (0, 0, 255), 1.6, 3)
        _text(img, "waiting for perception...", (max(view_w // 2 - 130, 8), h // 2 + 40))
        return img

    length, width, ball_r = _field_dims(state)
    x0, x1, y0, y1 = _bounds(state, zoom)
    P, _ = view_mapping(state, (view_w, h), zoom)
    scale = P.scale
    S = lambda m: max(int(round(m * scale)), 1)  # noqa: E731

    # ---- field, lines, goals ----
    cv2.rectangle(img, P(-length / 2, width / 2), P(length / 2, -width / 2), GRASS, -1)
    cv2.rectangle(img, P(-length / 2, width / 2), P(length / 2, -width / 2), LINE, 2)
    cv2.line(img, P(0, width / 2), P(0, -width / 2), LINE, 1, cv2.LINE_AA)
    cv2.circle(img, P(0, 0), 4, LINE, -1, cv2.LINE_AA)
    for side, g in (state.get("goals") or {}).items():
        depth = 0.25 * (1 if g["x"] > 0 else -1)
        cv2.rectangle(img, P(g["x"], g["y_max"]), P(g["x"] + depth, g["y_min"]), (0, 215, 255), 2)
        cv2.line(img, P(g["x"], g["y_max"]), P(g["x"], g["y_min"]), (0, 215, 255), 5)
        label = f"{side} goal"
        tx = P(g["x"] + depth, g["y_max"])[0] + (6 if g["x"] > 0 else -6 - 9 * len(label))
        _text(img, label, (min(max(tx, 4), view_w - 9 * len(label)), P(0, g["y_max"])[1] - 8), (0, 215, 255))
    # Axes hint.
    a = P(x0 + 0.08 * (x1 - x0), y0 + 0.08 * (y1 - y0))
    cv2.arrowedLine(img, a, (a[0] + 46, a[1]), (200, 200, 200), 1, cv2.LINE_AA, tipLength=0.25)
    cv2.arrowedLine(img, a, (a[0], a[1] - 46), (200, 200, 200), 1, cv2.LINE_AA, tipLength=0.25)
    _text(img, "+x", (a[0] + 50, a[1] + 5), (200, 200, 200), 0.45)
    _text(img, "+y", (a[0] - 8, a[1] - 52), (200, 200, 200), 0.45)

    # ---- floor tags: filled when readable in this frame, hollow when covered or out of view ----
    for tid, t in (state.get("floor_tags") or {}).items():
        c, half = P(t["x"], t["y"]), max(S(0.085), 5)
        box = ((c[0] - half, c[1] - half), (c[0] + half, c[1] + half))
        if t.get("visible"):
            cv2.rectangle(img, *box, (245, 245, 245), -1)
            cv2.rectangle(img, (c[0] - half // 2, c[1] - half // 2), (c[0] + half // 2, c[1] + half // 2), (20, 20, 20), -1)
        else:
            cv2.rectangle(img, *box, (150, 150, 150), 1)
        _text(img, f"tag {tid}", (c[0] + half + 4, c[1] + 4), (245, 245, 245) if t.get("visible") else (150, 150, 150), 0.45)

    ball = state.get("ball") or {}
    # ---- shot lines and robot-to-ball lines (under the objects) ----
    if ball.get("visible"):
        for name, rel in (state.get("relative") or {}).items():
            line = rel.get("shot_line")
            if line:
                d = rel["goal"]["distance"] + 0.5
                end = (ball["x"] + d * math.cos(line["ball_to_goal_yaw"]), ball["y"] + d * math.sin(line["ball_to_goal_yaw"]))
                _dashed(img, P(ball["x"], ball["y"]), P(*end), COLOURS.get(name, (200, 200, 200)), 1)
            r = (state.get("robots") or {}).get(name, {})
            if r.get("visible") and "ball" in rel:
                cv2.line(img, P(r["x"], r["y"]), P(ball["x"], ball["y"]), (255, 255, 0), 1, cv2.LINE_AA)

    # ---- the approach plan: path to the line-up point, then the run through the ball ----
    for name, ap in (state.get("approach") or {}).items():
        r, plan = (state.get("robots") or {}).get(name, {}), (ap or {}).get("plan")
        if not plan or not r.get("visible"):
            continue
        col = COLOURS.get(name, (200, 200, 200))
        c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
        world = lambda q: (r["x"] + c * q[0] - s * q[1], r["y"] + s * q[0] + c * q[1])  # noqa: E731  body -> field
        running = ap.get("phase") in ("RUN", "BRAKE")
        pts = [P(*world(q)) for q in plan["path"]]
        if not running:
            for a, b in zip(pts, pts[1:]):
                _dashed(img, a, b, (255, 255, 255), 2, 8)
        lu, end = P(*world(plan["line_up"])), P(*world(plan["run_end"]))
        cv2.circle(img, lu, max(S(0.05), 6), (255, 255, 255), 2, cv2.LINE_AA)
        cv2.arrowedLine(img, lu, end, col, 3 if running else 2, cv2.LINE_AA, tipLength=0.12)
        _text(img, "line-up", (lu[0] + 10, lu[1] + 18), (255, 255, 255), 0.45)

    # ---- robots ----
    for name, r in (state.get("robots") or {}).items():
        if not r.get("visible"):
            continue
        col = COLOURS.get(name, (200, 200, 200))
        c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
        corners = [(r["x"] + c * a - s * b, r["y"] + s * a + c * b)
                   for a, b in ((ROBOT_LEN / 2, ROBOT_WID / 2), (ROBOT_LEN / 2, -ROBOT_WID / 2),
                                (-ROBOT_LEN / 2, -ROBOT_WID / 2), (-ROBOT_LEN / 2, ROBOT_WID / 2))]
        poly = np.int32([P(*p) for p in corners])
        cv2.fillPoly(img, [poly], tuple(int(v * 0.45) for v in col), cv2.LINE_AA)
        cv2.polylines(img, [poly], True, col, 2, cv2.LINE_AA)
        nose = (r["x"] + c * (ROBOT_LEN / 2 + 0.12), r["y"] + s * (ROBOT_LEN / 2 + 0.12))
        cv2.arrowedLine(img, P(r["x"], r["y"]), P(*nose), col, 2, cv2.LINE_AA, tipLength=0.3)
        p = P(r["x"], r["y"])
        _text(img, name, (p[0] + 12, p[1] - S(ROBOT_WID) - 4), col, 0.6, 2)

    # ---- ball ----
    if ball.get("visible"):
        p = P(ball["x"], ball["y"])
        cv2.circle(img, p, max(S(ball_r), 5), PINK, -1, cv2.LINE_AA)
        cv2.circle(img, p, max(S(ball_r), 5), (255, 255, 255), 1, cv2.LINE_AA)
        speed = math.hypot(ball.get("vx", 0.0), ball.get("vy", 0.0))
        if speed > 0.05:  # where it will be in one second
            cv2.arrowedLine(img, p, P(ball["x"] + ball["vx"], ball["y"] + ball["vy"]), (255, 255, 255), 2,
                            cv2.LINE_AA, tipLength=0.2)
        _text(img, "ball", (p[0] + max(S(ball_r), 5) + 5, p[1] + 5), PINK, 0.6, 2)

    if not state.get("calibrated"):
        cv2.rectangle(img, (0, 0), (view_w, 56), (0, 0, 0), -1)
        _text(img, "NOT CALIBRATED: no positions yet", (16, 38), (0, 200, 255), 0.8, 2)
    if stale_s > 0.5:  # the picture is old: say so and dim it
        img[:, :view_w] = (img[:, :view_w] * 0.45).astype(np.uint8)
        _text(img, f"STALE: no update for {stale_s:.1f} s", (16, h - 24), (0, 0, 255), 0.9, 2)
    return img


def _sections(state: dict | None, stale_s: float) -> list[list[tuple]]:
    """The readouts as sections of rows: (text, colour, font scale, thickness, gap below).
    Sections: status + calibration + floor tags, the ball, then one per robot."""
    white, grey, dim = (225, 225, 225), (150, 150, 150), (130, 130, 130)
    head = lambda t: (t, grey, 0.45, 1, 22)  # noqa: E731
    if state is None:
        return [[("NO DATA", (0, 0, 255), 0.7, 2, 26), ("Start scripts/run_perception.py", (160, 160, 160), 0.5, 1, 24)]]

    first = [(f"STALE  {stale_s:.1f} s since last update", (0, 0, 255), 0.6, 2, 28) if stale_s > 0.5
             else ("LIVE", (0, 220, 0), 0.6, 2, 28), head("CALIBRATION")]
    tags = state.get("floor_tags") or {}
    if state.get("calibrated"):
        mode = state.get("calib_mode")
        how = {"tags": "measured floor tags", "survey": "tags in view (unmeasured)", "clicks": "clicked points",
               "corners": "corner tags define the field", "fake": "FAKE FIELD, no camera"}
        first.append((f"done: {how.get(mode, mode or '?')}", white, 0.52, 1, 24))
        if mode == "corners":
            fld = state.get("field") or {}
            first.append((f"field {fld.get('length', 0):.2f} x {fld.get('width', 0):.2f} m, from the tags", (0, 220, 0), 0.5, 1, 24))
        if mode == "survey":  # the field here is only a drawing placed on a tag, not the real arena
            ref = next((tid for tid, t in tags.items() if abs(t["x"]) < 1e-9 and abs(t["y"]) < 1e-9), "?")
            first.append((f"field is drawn around tag {ref}", (0, 200, 255), 0.5, 1, 24))
        err = state.get("calib_err_px")
        bad = err is not None and err > 8
        first.append((f"error {err:.1f} px" + ("  MOVED: recalibrate" if bad else "") if err is not None else "error unknown",
                      (0, 0, 255) if bad else white, 0.52, 1, 24))
    else:
        first.append(("NOT CALIBRATED", (0, 200, 255), 0.6, 2, 24))
    first.append((f"frame age {state.get('frame_age_s', 0) * 1000:.0f} ms", white, 0.52, 1, 30))
    first.append(head("FLOOR TAGS"))
    first.append((f"readable now: {sum(1 for t in tags.values() if t.get('visible'))} of {len(tags)}"
                  if tags else "none in the layout", white, 0.52, 1, 24))
    hidden = [tid for tid in sorted(tags, key=lambda k: int(k)) if not tags[tid].get("visible")]
    if hidden:
        first.append(("hidden or out of view: " + ", ".join(hidden), dim, 0.47, 1, 24))

    ball = state.get("ball") or {}
    ball_rows = [head("BALL")]
    if ball.get("visible"):
        ball_rows.append((f"({ball['x'] * 100:+.0f}, {ball['y'] * 100:+.0f}) cm", PINK, 0.6, 2, 24))
        ball_rows.append((f"speed {math.hypot(ball['vx'], ball['vy']):.2f} m/s", white, 0.52, 1, 24))
        ball_rows.append((f"age {ball['age_s'] * 1000:.0f} ms", grey, 0.45, 1, 30))
    else:
        ball_rows.append(("not seen", dim, 0.52, 1, 30))

    out = [first, ball_rows]
    for name, r in (state.get("robots") or {}).items():
        col = COLOURS.get(name, (200, 200, 200))
        rows = [head(name.upper())]
        if not r.get("visible"):
            rows.append(("not seen", dim, 0.52, 1, 30))
            out.append(rows)
            continue
        rows.append((f"({r['x'] * 100:+.0f}, {r['y'] * 100:+.0f}) cm  facing {math.degrees(r['yaw']):+.0f} deg", col, 0.54, 2, 24))
        rel = (state.get("relative") or {}).get(name, {})
        if "ball" in rel:
            b = rel["ball"]
            side = "left" if b["bearing"] >= 0 else "right"
            rows.append((f"ball {b['distance'] * 100:.0f} cm, turn {abs(math.degrees(b['bearing'])):.0f} deg {side}", white, 0.52, 1, 22))
            rows.append((f"  {b['x'] * 100:+.0f} cm ahead, {b['y'] * 100:+.0f} cm left", white, 0.52, 1, 22))
        if "shot_line" in rel:
            sl = rel["shot_line"]
            rows.append((f"shot line: {sl['behind_ball'] * 100:+.0f} cm behind ball,", white, 0.52, 1, 20))
            rows.append((f"  {sl['left_of_line'] * 100:+.0f} cm left of it, turn {math.degrees(sl['turn_to_shot_line']):+.0f} deg",
                         white, 0.52, 1, 22))
        shot = (state.get("shots") or {}).get(name, {})
        if "ready" in shot:
            rows.append(("READY TO KICK", (0, 220, 0), 0.52, 2, 24) if shot["ready"]
                        else ("not in kick position", (170, 170, 170), 0.52, 1, 24))
        ap = (state.get("approach") or {}).get(name)
        if ap:
            phase_col = {"APPROACH": (0, 200, 255), "ALIGN": (0, 255, 255), "RUN": (0, 220, 0), "BRAKE": (170, 170, 170)}
            rows.append((f"PLAN: {ap['phase']}", phase_col.get(ap["phase"], white), 0.56, 2, 22))
            cmd = ap["cmd"]
            rows.append((f"cmd  fwd {cmd['vx']:+.2f}  left {cmd['vy']:+.2f} m/s  turn {cmd['wz']:+.2f}", white, 0.47, 1, 22))
            for k, step in enumerate(((ap.get("plan") or {}).get("steps") or [])[:5], 1):
                rows.append((f"{k}. {step}"[:46], (200, 200, 200), 0.45, 1, 19))
        rows.append((f"age {r['age_s'] * 1000:.0f} ms", grey, 0.45, 1, 30))
        out.append(rows)
    return out


def _draw_rows(img, rows, x, y):
    for text, colour, scale, thick, gap in rows:
        _text(img, text, (x, y), colour, scale, thick)
        y += gap
    return y


def render_panel(state: dict | None, size=(PANEL_W, 800), stale_s: float = 0.0, zoom: str = "field") -> np.ndarray:
    """The readouts as one tall column (the standalone dashboard's right-hand side)."""
    w, h = size
    img = np.full((h, w, 3), (18, 18, 18), np.uint8)
    cv2.line(img, (0, 0), (0, h), (70, 70, 70), 1)
    _text(img, "Pupper FC field view", (16, 34), (255, 255, 255), 0.75, 2)
    y = 68
    for rows in _sections(state, stale_s):
        y = _draw_rows(img, rows, 16, y) + 6
    _text(img, f"z = zoom ({'fit to objects' if zoom == 'fit' else 'whole field'})    q = quit",
          (16, h - 16), (150, 150, 150), 0.45)
    return img


def render_strip(state: dict | None, size=(1400, 250), stale_s: float = 0.0) -> np.ndarray:
    """The same readouts laid out as columns, to sit underneath the two views."""
    w, h = size
    sections = _sections(state, stale_s)
    h = max(h, 30 + max(sum(row[4] for row in rows) for rows in sections))  # grow to fit the longest column
    img = np.full((h, w, 3), (18, 18, 18), np.uint8)
    cv2.line(img, (0, 0), (w, 0), (70, 70, 70), 1)
    col_w = w // max(len(sections), 4)
    for k, rows in enumerate(sections):
        if k:
            cv2.line(img, (k * col_w, 10), (k * col_w, h - 10), (50, 50, 50), 1)
        _draw_rows(img, rows, k * col_w + 16, 30)
    return img


def render(state: dict | None, size=(1280, 800), zoom: str = "field", stale_s: float = 0.0) -> np.ndarray:
    """The standalone dashboard: field view on the left, readouts on the right."""
    w, h = size
    return np.hstack([render_field(state, (w - PANEL_W, h), zoom, stale_s),
                      render_panel(state, (PANEL_W, h), stale_s, zoom)])


def render_combined(camera: np.ndarray, state: dict | None, width: int = 1400, zoom: str = "field",
                    stale_s: float = 0.0, camera_share: float = 0.62):
    """One picture for the live program: camera feed (left) beside the field view (right),
    readouts underneath. Returns (picture, camera_scale, (camera_w, camera_h)); the camera
    sits at the top-left corner, so a window pixel (x, y) inside it is camera pixel
    (x / camera_scale, y / camera_scale)."""
    cam_w = int(width * camera_share)
    k = cam_w / camera.shape[1]
    cam_h = int(round(camera.shape[0] * k))
    cam = cv2.resize(camera, (cam_w, cam_h), interpolation=cv2.INTER_AREA)
    top = np.hstack([cam, render_field(state, (width - cam_w, cam_h), zoom, stale_s)])
    return np.vstack([top, render_strip(state, (width, 250), stale_s)]), k, (cam_w, cam_h)
