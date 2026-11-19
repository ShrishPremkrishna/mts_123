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


def _dashed(img, a, b, colour, thick=1, dash=10):
    a, b = np.array(a, float), np.array(b, float)
    n = max(int(np.linalg.norm(b - a) // dash), 1)
    for k in range(0, n, 2):
        p, q = a + (b - a) * k / n, a + (b - a) * min(k + 1, n) / n
        cv2.line(img, tuple(np.int32(p)), tuple(np.int32(q)), colour, thick, cv2.LINE_AA)


def _text(img, text, org, colour=(235, 235, 235), scale=0.55, thick=1):
    cv2.putText(img, text, (int(org[0]), int(org[1])), FONT, scale, colour, thick, cv2.LINE_AA)


def render(state: dict | None, size=(1280, 800), zoom: str = "field", stale_s: float = 0.0) -> np.ndarray:
    """state: a perception state (None = nothing received yet). zoom: "field" or "fit".
    stale_s: how long ago this state arrived; the view greys out when it is old."""
    w, h = size
    img = np.full((h, w, 3), (28, 28, 28), np.uint8)
    view_w = w - PANEL_W
    if state is None:
        _text(img, "NO DATA", (view_w // 2 - 110, h // 2), (0, 0, 255), 1.6, 3)
        _text(img, "waiting for the perception program...", (view_w // 2 - 190, h // 2 + 40))
        _panel(img, view_w, None, stale_s, zoom)
        return img

    length, width, ball_r = _field_dims(state)
    x0, x1, y0, y1 = _bounds(state, zoom)
    scale = min((view_w - 40) / (x1 - x0), (h - 40) / (y1 - y0))
    ox = 20 + ((view_w - 40) - scale * (x1 - x0)) / 2
    oy = 20 + ((h - 40) - scale * (y1 - y0)) / 2
    P = lambda x, y: (int(round(ox + (x - x0) * scale)), int(round(oy + (y1 - y) * scale)))  # noqa: E731
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
        _text(img, "NOT CALIBRATED: no positions yet", (16, 38), (0, 200, 255), 1.0, 2)
    if stale_s > 0.5:  # the picture is old: say so and dim it
        img[:, :view_w] = (img[:, :view_w] * 0.45).astype(np.uint8)
        _text(img, f"STALE: no update for {stale_s:.1f} s", (16, h - 24), (0, 0, 255), 1.0, 2)
    _panel(img, view_w, state, stale_s, zoom)
    return img


def _panel(img, x0, state, stale_s, zoom):
    h = img.shape[0]
    cv2.rectangle(img, (x0, 0), (img.shape[1], h), (18, 18, 18), -1)
    cv2.line(img, (x0, 0), (x0, h), (70, 70, 70), 1)
    y = [34]

    def row(text, colour=(225, 225, 225), scale=0.52, thick=1, gap=24):
        _text(img, text, (x0 + 16, y[0]), colour, scale, thick)
        y[0] += gap

    row("Pupper FC field view", (255, 255, 255), 0.75, 2, 34)
    if state is None:
        row("NO DATA", (0, 0, 255), 0.7, 2)
        row("Start scripts/run_perception.py", (160, 160, 160))
        return
    if stale_s > 0.5:
        row(f"STALE  {stale_s:.1f} s since last update", (0, 0, 255), 0.6, 2, 30)
    else:
        row("LIVE", (0, 220, 0), 0.6, 2, 30)

    row("CALIBRATION", (150, 150, 150), 0.45)
    if state.get("calibrated"):
        how = {"tags": "measured floor tags", "survey": "tags in view, nothing measured", "clicks": "clicked points"}
        row(f"done: {how.get(state.get('calib_mode'), state.get('calib_mode') or '?')}")
        err = state.get("calib_err_px")
        bad = err is not None and err > 8
        row(f"error {err:.1f} px" + ("  CAMERA OR TAGS MOVED" if bad else "") if err is not None else "error unknown",
            (0, 0, 255) if bad else (225, 225, 225))
    else:
        row("NOT CALIBRATED", (0, 200, 255), 0.6, 2)
    row(f"frame age {state.get('frame_age_s', 0) * 1000:.0f} ms", gap=32)

    tags = state.get("floor_tags") or {}
    seen = sorted(tags, key=lambda k: int(k))
    row("FLOOR TAGS", (150, 150, 150), 0.45)
    row(f"readable now: {sum(1 for t in tags.values() if t.get('visible'))} of {len(tags)}"
        if tags else "none in the layout")
    hidden = [tid for tid in seen if not tags[tid].get("visible")]
    if hidden:
        row("hidden or out of view: " + ", ".join(hidden), (130, 130, 130), 0.47)
    y[0] += 8

    ball = state.get("ball") or {}
    row("BALL", (150, 150, 150), 0.45)
    if ball.get("visible"):
        row(f"({ball['x'] * 100:+.0f}, {ball['y'] * 100:+.0f}) cm", PINK, 0.6, 2)
        row(f"speed {math.hypot(ball['vx'], ball['vy']):.2f} m/s   age {ball['age_s'] * 1000:.0f} ms", gap=32)
    else:
        row("not seen", (130, 130, 130), gap=32)

    for name, r in (state.get("robots") or {}).items():
        col = COLOURS.get(name, (200, 200, 200))
        row(name.upper(), (150, 150, 150), 0.45)
        if not r.get("visible"):
            row("not seen", (130, 130, 130), gap=32)
            continue
        row(f"({r['x'] * 100:+.0f}, {r['y'] * 100:+.0f}) cm  facing {math.degrees(r['yaw']):+.0f} deg", col, 0.56, 2)
        rel = (state.get("relative") or {}).get(name, {})
        if "ball" in rel:
            b = rel["ball"]
            side = "left" if b["bearing"] >= 0 else "right"
            row(f"ball {b['distance'] * 100:.0f} cm, turn {abs(math.degrees(b['bearing'])):.0f} deg {side}")
            row(f"  {b['x'] * 100:+.0f} cm ahead, {b['y'] * 100:+.0f} cm left", gap=22)
        if "shot_line" in rel:
            s = rel["shot_line"]
            row(f"shot line: {s['behind_ball'] * 100:+.0f} cm behind ball,", gap=20)
            row(f"  {s['left_of_line'] * 100:+.0f} cm left of it, turn {math.degrees(s['turn_to_shot_line']):+.0f} deg", gap=22)
        shot = (state.get("shots") or {}).get(name, {})
        if "ready" in shot:
            row("READY TO KICK" if shot["ready"] else "not in kick position",
                (0, 220, 0) if shot["ready"] else (170, 170, 170), 0.52, 2 if shot["ready"] else 1)
        row(f"age {r['age_s'] * 1000:.0f} ms", (150, 150, 150), 0.45, gap=30)

    _text(img, f"z = zoom ({'fit to objects' if zoom == 'fit' else 'whole field'})    q = quit",
          (x0 + 16, h - 16), (150, 150, 150), 0.45)
