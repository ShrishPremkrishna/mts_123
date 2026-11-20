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


def _label(img, used: list, text, org, colour=(235, 235, 235), scale=0.55, thick=1):
    """Text that stays inside the picture and steps down out of the way of labels already drawn."""
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, thick)
    h, w = img.shape[:2]
    x = int(min(max(org[0], 2), max(w - tw - 2, 2)))
    y = int(min(max(org[1], th + 2), h - 4))
    for _ in range(10):
        box = (x - 2, y - th - 2, x + tw + 2, y + 4)
        if not any(box[0] < u[2] and u[0] < box[2] and box[1] < u[3] and u[1] < box[3] for u in used):
            break
        y = min(y + th + 6, h - 4)
    used.append((x - 2, y - th - 2, x + tw + 2, y + 4))
    cv2.putText(img, text, (x, y), FONT, scale, (20, 20, 20), thick + 2, cv2.LINE_AA)   # dark edge: readable on grass
    cv2.putText(img, text, (x, y), FONT, scale, colour, thick, cv2.LINE_AA)


BALL_COLOURS = {"NOT_SEEN": (130, 130, 130), "HIDDEN": (255, 170, 60), "STILL": (225, 225, 225),
                "HIT": (0, 140, 255), "ROLLING": PINK, "KNOCKED": (0, 230, 255)}
STATE_COLOURS = {"HOLD": (130, 130, 130), "SEARCH": (130, 130, 130), "WAIT": (170, 170, 170),
                 "GO_TO_KICK_POSE": (255, 200, 0), "WAIT_FOR_BALL": (0, 230, 255), "WIND_UP": (0, 255, 0),
                 "KICKING": (0, 255, 0), "BLOCK": (0, 0, 255), "GUARD": (0, 140, 255)}
ROLE_COLOURS = {"OFFENSE": (0, 220, 0), "DEFENSE": (0, 120, 255)}


def _outline(img, P, pose, colour, thick):
    """The robot's footprint at a pose, nose marked: where it has to stand."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    box = np.int32([P(x + c * a - s * b, y + s * a + c * b)
                    for a, b in ((ROBOT_LEN / 2, ROBOT_WID / 2), (ROBOT_LEN / 2, -ROBOT_WID / 2),
                                 (-ROBOT_LEN / 2, -ROBOT_WID / 2), (-ROBOT_LEN / 2, ROBOT_WID / 2))])
    cv2.polylines(img, [box], True, colour, thick, cv2.LINE_AA)
    cv2.line(img, P(x + c * ROBOT_LEN / 2, y + s * ROBOT_LEN / 2), P(x + c * (ROBOT_LEN / 2 + 0.08), y + s * (ROBOT_LEN / 2 + 0.08)),
             colour, thick, cv2.LINE_AA)


def _route(img, used, P, path, colour):
    """A route as the robot will walk it: dashed legs, numbered corners, each leg's length beside it."""
    pts = [P(*q) for q in path]
    for k, ((ax, ay), (bx, by)) in enumerate(zip(path, path[1:])):
        _dashed(img, pts[k], pts[k + 1], colour, 2, 8)
        length = math.hypot(bx - ax, by - ay)
        if length > 0.15:
            mid = ((pts[k][0] + pts[k + 1][0]) // 2 + 6, (pts[k][1] + pts[k + 1][1]) // 2 - 6)
            _label(img, used, f"{k + 1}: {length:.2f} m", mid, colour, 0.42)
    for k, p in enumerate(pts[1:-1], 1):
        cv2.circle(img, p, 5, colour, -1, cv2.LINE_AA)
        cv2.circle(img, p, 5, (20, 20, 20), 1, cv2.LINE_AA)


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

    used: list = []    # screen rectangles already holding a label
    # ---- floor tags: filled when readable in this frame, hollow when covered or out of view ----
    for tid, t in (state.get("floor_tags") or {}).items():
        c, half = P(t["x"], t["y"]), max(S(0.085), 5)
        box = ((c[0] - half, c[1] - half), (c[0] + half, c[1] + half))
        if t.get("visible"):
            cv2.rectangle(img, *box, (245, 245, 245), -1)
            cv2.rectangle(img, (c[0] - half // 2, c[1] - half // 2), (c[0] + half // 2, c[1] + half // 2), (20, 20, 20), -1)
        else:
            cv2.rectangle(img, *box, (150, 150, 150), 1)
        _label(img, used, f"tag {tid}", (c[0] + half + 4, c[1] + 4), (245, 245, 245) if t.get("visible") else (150, 150, 150), 0.45)

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
        _label(img, used, "line-up", (lu[0] + 10, lu[1] + 18), (255, 255, 255), 0.45)

    # ---- where the ball is going: dashed path, a tick each second, a ring where it ends ----
    pred = state.get("prediction") or {}
    if pred.get("moving") and pred.get("path"):
        pts = [P(x, y) for _, x, y in pred["path"]]
        for a, b in zip(pts, pts[1:]):
            _dashed(img, a, b, PINK, 2, 7)
        for (t0, _, _), (t1, x, y) in zip(pred["path"], pred["path"][1:]):
            if int(t1) > int(t0):                          # crossed a whole second
                cv2.circle(img, P(x, y), 3, (255, 255, 255), -1, cv2.LINE_AA)
        end = P(*pred["end"])
        area = pred.get("area")
        if area:
            # The stopping area: where the ball should come to rest, about 2 sigma. It shrinks as the roll goes on.
            axes, tilt = (max(S(area["along"]), 2), max(S(area["across"]), 2)), -math.degrees(area["heading"])
            shade = img.copy()
            cv2.ellipse(shade, end, axes, tilt, 0, 360, PINK, -1, cv2.LINE_AA)
            img[:] = cv2.addWeighted(shade, 0.28, img, 0.72, 0)
            cv2.ellipse(img, end, axes, tilt, 0, 360, PINK, 1, cv2.LINE_AA)
        _draw_trail(img, P, pred.get("trail") or [], (255, 255, 255))
        cv2.circle(img, end, max(S(ball_r), 5), PINK, 2, cv2.LINE_AA)
        _label(img, used, _ending(pred), (end[0] + max(S(ball_r), 5) + 5, end[1] - 6), PINK, 0.5, 1)
    done = pred.get("last_roll")
    if done:
        # Just after a roll: what each frame had predicted (dots), against where the ball really stopped.
        _draw_trail(img, P, done["trail"], (170, 170, 170))
        c = P(*done["stopped"])
        _label(img, used, f"first guess was {done['first_guess_error'] * 100:.0f} cm off", (c[0] + max(S(ball_r), 5) + 6, c[1] - 14),
              (200, 200, 200), 0.45)

    # ---- what each robot should do: its active plan drawn step by step ----
    machines = (state.get("machines") or {}).get("robots") or {}
    for name, plans in (state.get("intercepts") or {}).items():
        r = (state.get("robots") or {}).get(name, {})
        if not r.get("visible"):
            continue
        col = COLOURS.get(name, (200, 200, 200))
        mach = machines.get(name) or {}
        strike, kick = plans.get("strike") or {}, plans.get("kick") or {}
        block, guard = plans.get("block"), plans.get("guard") or {}
        own_drawing = name in (state.get("approach") or {})     # the fake approach draws its own run-up
        active = {"BLOCK": "block", "GUARD": "guard"}.get(mach.get("state"),
                                                           "kick" if kick.get("feasible") else "strike")
        if active == "strike" and strike.get("feasible") and not own_drawing:
            _route(img, used, P, strike.get("path") or [], col)
            x, y, yaw = strike["pose"]
            cv2.circle(img, P(x, y), max(S(0.05), 6), col, 2, cv2.LINE_AA)
            cv2.arrowedLine(img, P(x, y), P(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw)), col, 2, cv2.LINE_AA, tipLength=0.3)
            _label(img, used, f"line up, {strike['travel_s']:.1f} s" + (" (then wait)" if strike.get("hold") else ""),
                   (P(x, y)[0] + 10, P(x, y)[1] + 18), col, 0.45)
        if kick.get("feasible") and active == "kick":            # only the plan this robot is acting on
            green = (0, 255, 0)
            if not own_drawing:
                _route(img, used, P, kick.get("path") or [], col)
            if not kick["sure"]:
                _outline(img, P, kick["stage"], green, 1)        # where to wait while the stop is uncertain
                _label(img, used, "wait here", (P(*kick["stage"][:2])[0] - 30, P(*kick["stage"][:2])[1] - S(ROBOT_WID) - 4),
                       green, 0.42)
            _outline(img, P, kick["pose"], green, 2)             # the kick pose itself
            x, y, _ = kick["pose"]
            what = {"WIND_UP": "KICK NOW", "KICKING": "KICKING"}.get(mach.get("state"), "kick pose")
            _label(img, used, f"{what}: {kick['paw']} paw, contact in {kick['contact_in_s']:.1f} s",
                   (P(x, y)[0] - 40, P(x, y)[1] + S(ROBOT_WID) + 14), green, 0.45,
                   2 if mach.get("state") in ("WIND_UP", "KICKING") else 1)
        if active == "guard" and guard.get("pose"):
            _route(img, used, P, guard.get("path") or [], col)
            _outline(img, P, guard["pose"], (0, 140, 255), 2)
            _label(img, used, f"guard, {guard['travel_s']:.1f} s", (P(*guard["pose"][:2])[0] + 12, P(*guard["pose"][:2])[1] - 12),
                   (0, 140, 255), 0.45)
        if block and block.get("point"):
            red = (0, 0, 255)
            _route(img, used, P, block.get("path") or [], red)
            c = P(*block["point"])
            cv2.drawMarker(img, c, red, cv2.MARKER_TILTED_CROSS, 18, 3, cv2.LINE_AA)
            _label(img, used, "block: " + _spare(block), (c[0] + 12, c[1] + 22), red, 0.5, 2)

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
        _label(img, used, name, (p[0] + 12, p[1] - S(ROBOT_WID) - 4), col, 0.6, 2)
        mach = ((state.get("machines") or {}).get("robots") or {}).get(name)
        if mach:
            _label(img, used, f"{mach['role']}: {mach['state'].replace('_', ' ')}", (p[0] + 12, p[1] - S(ROBOT_WID) + 14),
                   STATE_COLOURS.get(mach["state"], (200, 200, 200)), 0.45, 1)

    # ---- ball ----
    if ball.get("visible"):
        p = P(ball["x"], ball["y"])
        cv2.circle(img, p, max(S(ball_r), 5), PINK, -1, cv2.LINE_AA)
        cv2.circle(img, p, max(S(ball_r), 5), (255, 255, 255), 1, cv2.LINE_AA)
        speed = math.hypot(ball.get("vx", 0.0), ball.get("vy", 0.0))
        if speed > 0.05 and "moving" not in pred:  # no prediction published: where it would be in one second
            cv2.arrowedLine(img, p, P(ball["x"] + ball["vx"], ball["y"] + ball["vy"]), (255, 255, 255), 2,
                            cv2.LINE_AA, tipLength=0.2)
        ball_state = ((state.get("machines") or {}).get("ball") or {}).get("state")
        _label(img, used, "ball" + (f": {ball_state.replace('_', ' ')}" if ball_state else ""),
               (p[0] + max(S(ball_r), 5) + 5, p[1] + 5), BALL_COLOURS.get(ball_state, PINK), 0.6, 2)

    if not state.get("calibrated"):
        cv2.rectangle(img, (0, 0), (view_w, 56), (0, 0, 0), -1)
        _label(img, used, "NOT CALIBRATED: no positions yet", (16, 38), (0, 200, 255), 0.8, 2)
    if stale_s > 0.5:  # the picture is old: say so and dim it
        img[:, :view_w] = (img[:, :view_w] * 0.45).astype(np.uint8)
        _label(img, used, f"STALE: no update for {stale_s:.1f} s", (16, h - 24), (0, 0, 255), 0.9, 2)
    return img


def _draw_trail(img, P, trail, colour):
    """Where earlier frames said the ball would stop: a thin line through them, newer dots larger."""
    pts = [P(x, y) for _, x, y in trail]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, colour, 1, cv2.LINE_AA)
    for k, p in enumerate(pts):
        cv2.circle(img, p, 2 if k < len(pts) - 5 else 3, colour, -1, cv2.LINE_AA)


def _ending(pred: dict) -> str:
    """How the predicted roll ends, in a few words."""
    t = pred.get("end_in_s", 0.0)
    return {"stops": f"stops in {t:.1f} s", "out": f"out in {t:.1f} s", "horizon": "still rolling",
            "goal_east": f"EAST GOAL in {t:.1f} s", "goal_west": f"WEST GOAL in {t:.1f} s"}.get(pred.get("ends"), "")


def _spare(block: dict) -> str:
    """How much time a block leaves, in a few words."""
    spare = block["spare_s"]
    if spare < 0:
        return f"{-spare:.1f} s too late"
    return f"{spare:.1f} s to spare" if block["feasible"] else f"tight, {spare:.1f} s to spare"


def _wrap(text: str, scale: float, thick: int, max_px: int) -> list[str]:
    """Break a line at spaces so each piece fits max_px; continuation lines are indented."""
    lines, line = [], ""
    for word in text.split(" "):
        trial = f"{line} {word}" if line else word
        if line and cv2.getTextSize(trial, FONT, scale, thick)[0][0] > max_px:
            lines.append(line)
            line = "   " + word
        else:
            line = trial
    return lines + [line]


def _sections(state: dict | None, stale_s: float, detail: bool = True, max_px: int = 320) -> list[list[tuple]]:
    """The readouts as sections of rows: (text, colour, font scale, thickness, gap below).
    Sections: system + calibration, the ball, then one per robot. detail=False keeps only the headline rows."""
    white, grey, dim, warn = (225, 225, 225), (150, 150, 150), (130, 130, 130), (0, 200, 255)
    head = lambda t: (t, grey, 0.45, 1, 22)  # noqa: E731

    def add(rows, text, colour=white, scale=0.47, thick=1, gap=20, last_gap=None):
        pieces = _wrap(text, scale, thick, max_px)
        for k, piece in enumerate(pieces):
            rows.append((piece, colour, scale, thick, (last_gap or gap) if k == len(pieces) - 1 else gap - 2))

    if state is None:
        return [[("NO DATA", (0, 0, 255), 0.7, 2, 26), ("Start scripts/run_perception.py", (160, 160, 160), 0.5, 1, 24)]]

    diag = state.get("diagnostics") or {}
    # ---------------- system ----------------
    first = [(f"STALE  {stale_s:.1f} s since last update", (0, 0, 255), 0.6, 2, 26) if stale_s > 0.5
             else ("LIVE", (0, 220, 0), 0.6, 2, 26), head("CALIBRATION")]
    tags = state.get("floor_tags") or {}
    status, reason = state.get("calibration_status"), state.get("calibration_reason")
    if state.get("calibrated"):
        mode = state.get("calib_mode")
        how = {"tags": "measured floor tags", "survey": "tags in view (unmeasured)", "clicks": "clicked points",
               "corners": "corner tags define the field", "fake": "FAKE FIELD, no camera"}
        add(first, f"valid: {how.get(mode, mode or '?')}", (0, 220, 0), 0.5)
        fld = state.get("field") or {}
        if mode == "corners":
            add(first, f"field {fld.get('length', 0):.2f} x {fld.get('width', 0):.2f} m, measured from the tags")
        if mode == "survey":  # the field here is only a drawing placed on a tag, not the real arena
            ref = next((tid for tid, t in tags.items() if abs(t["x"]) < 1e-9 and abs(t["y"]) < 1e-9), "?")
            add(first, f"field is only drawn around tag {ref}", warn)
    elif status == "invalidated" or reason:
        add(first, "WITHHELD: recalibrate (f)", (0, 0, 255), 0.55, 2)
        add(first, str(reason or "calibration needs verification").replace("_", " "), warn)
    else:
        add(first, "NOT CALIBRATED", warn, 0.55, 2)
    drift, limit = state.get("calib_err_px"), diag.get("drift_limit_px", 8.0)
    if drift is not None:
        close = drift > 0.75 * limit
        add(first, f"tag drift {drift:.1f} px of {limit:.0f} allowed" + ("  (close to the limit)" if close else ""),
            (0, 0, 255) if drift > limit else warn if close else white, last_gap=22)
    if detail:
        if diag.get("camera_height") is not None:
            add(first, f"camera {diag['camera_height']:.2f} m up, fit {diag['fit_px']:.1f} px", grey, 0.45)
        if diag.get("lens_deg") is not None:
            add(first, f"lens {diag['lens_deg']:.1f} deg across: {diag['lens_from']}", grey, 0.45)
        elif diag.get("lens_from"):
            add(first, f"lens: {diag['lens_from']}", grey, 0.45)
    first.append(head("FRAMES"))
    fps = diag.get("fps")
    add(first, (f"{fps:.0f} per second, " if fps else "") + f"age {state.get('frame_age_s', 0) * 1000:.0f} ms",
        warn if fps and fps < 10 else white, last_gap=22)
    first.append(head("TAGS"))
    seen = [tid for tid in sorted(tags, key=lambda k: int(k)) if tags[tid].get("visible")]
    hidden = [tid for tid in sorted(tags, key=lambda k: int(k)) if not tags[tid].get("visible")]
    add(first, (f"floor: {', '.join(map(str, seen)) or 'none'} readable" if tags else "floor: none in the layout")
        + (f"; hidden: {', '.join(map(str, hidden))}" if hidden else ""))
    if diag.get("ignored_tags"):
        add(first, f"ignored (not on the floor): {', '.join(map(str, diag['ignored_tags']))}", warn)
    if "robot_tags_seen" in diag:
        add(first, "robots: " + (", ".join(diag["robot_tags_seen"]) or "none seen"))
    if diag.get("problem"):
        add(first, "PREDICTION FAULT (tracking continues):", (0, 0, 255), 0.47, 2)
        add(first, diag["problem"], (0, 0, 255), 0.42)

    # ---------------- ball ----------------
    ball, pred = state.get("ball") or {}, state.get("prediction")
    machine = (state.get("machines") or {}).get("ball")
    ball_rows = [head("BALL")]
    if machine:
        ball_rows.append((f"{machine['state'].replace('_', ' ')}  {machine['since_s']:.1f} s", BALL_COLOURS.get(machine["state"], white), 0.62, 2, 24))
        add(ball_rows, machine["why"], white)
        if machine.get("going"):
            add(ball_rows, machine["going"], PINK, 0.5, 1, 20, 24)
    if ball.get("visible") or pred:
        here = pred or ball
        add(ball_rows, f"at ({here['x'] * 100:+.0f}, {here['y'] * 100:+.0f}) cm", PINK, 0.55, 2, 22)
        if pred is None:
            add(ball_rows, f"speed {math.hypot(ball['vx'], ball['vy']):.2f} m/s")
        elif pred["moving"]:
            area = pred.get("area")
            add(ball_rows, f"speed {pred['speed']:.2f} m/s, heading {math.degrees(pred['heading']):+.0f} deg")
            if area:
                add(ball_rows, f"stopping area {area['along'] * 200:.0f} x {area['across'] * 200:.0f} cm (long x wide)")
            if pred.get("bounces"):
                add(ball_rows, f"{pred['bounces']} bounce(s) on the way")
        if pred and detail:
            fit = pred.get("fit") or {}
            if pred["moving"]:
                phase = {"starting": "too few frames yet: going by the starting guess",
                         "early": "first 0.6 s: speed from the newest frames, slow-down held at the guess",
                         "own_rate": "fitting this roll's own slow-down"}.get(fit.get("phase"), "")
                add(ball_rows, f"this roll: {fit.get('roll_frames', 0)} frames over {fit.get('roll_s', 0):.1f} s", grey, 0.45)
                if phase:
                    add(ball_rows, phase, grey, 0.45)
                add(ball_rows, f"slowing at {pred['decel']:.2f} +/- {pred.get('decel_sigma', 0):.2f} m/s2", grey, 0.45)
                if fit.get("knocks"):
                    add(ball_rows, f"knocked off course {fit['knocks']}x, turned {fit['turned_deg']:.0f} deg", warn, 0.45)
            if fit:
                add(ball_rows, f"starting guess {fit['guess']:.2f} +/- {fit['guess_spread']:.2f} from {fit['guess_from']}", grey, 0.45)
                add(ball_rows, f"position noise {fit['noise'] * 100:.1f} cm, drift allowance {fit['wander_deg']:.0f} deg", grey, 0.45)
            if pred.get("hidden"):
                add(ball_rows, f"NOT SEEN for {pred['seen_ago_s']:.1f} s: predicted, not measured", warn, 0.45)
        if pred and pred.get("last_roll"):
            done = pred["last_roll"]
            add(ball_rows, f"last roll: first guess {done['first_guess_error'] * 100:.0f} cm off"
                + (f", slowed at {done['decel']:.2f}" if done.get("decel") else ""), grey, 0.45)
        rate = diag.get("ball_seen_rate")
        if rate is not None and rate < 0.8:
            add(ball_rows, f"! found in only {rate * 100:.0f}% of frames: set its colour (cursor on the ball, press b)", warn, 0.45)
        if ball.get("visible"):
            add(ball_rows, f"age {ball['age_s'] * 1000:.0f} ms", grey, 0.42, 1, 20, 26)
    elif not machine:
        ball_rows.append(("not seen", dim, 0.52, 1, 30))

    # ---------------- robots ----------------
    out = [first, ball_rows]
    for name, r in (state.get("robots") or {}).items():
        col = COLOURS.get(name, (200, 200, 200))
        rows = [head(name.upper())]
        mach = ((state.get("machines") or {}).get("robots") or {}).get(name)
        plans = (state.get("intercepts") or {}).get(name) or {}
        if mach:
            rows.append((f"{mach['role']}", ROLE_COLOURS.get(mach["role"], white), 0.58, 2, 20))
            add(rows, mach["role_why"], grey, 0.45, 1, 18, 22)
            rows.append((f"{mach['state'].replace('_', ' ')}  {mach['since_s']:.1f} s", STATE_COLOURS.get(mach["state"], white), 0.58, 2, 20))
            add(rows, mach["why"], white, 0.47, 1, 19, 22)
            if mach.get("decision"):
                add(rows, {"intercept": "DECISION: INTERCEPT. Wind-up starts before the ball stops",
                           "wait_for_stop": "DECISION: WAIT FOR STOP. Stopping point not sure enough yet"}[mach["decision"]],
                    (0, 255, 0) if mach["decision"] == "intercept" else (0, 230, 255), 0.47, 2, 19, 22)
        if not r.get("visible"):
            rows.append(("tag not seen", dim, 0.5, 1, 26))
            out.append(rows)
            continue
        add(rows, f"at ({r['x'] * 100:+.0f}, {r['y'] * 100:+.0f}) cm, facing {math.degrees(r['yaw']):+.0f} deg", col, 0.5, 2, 20, 22)
        if plans.get("stale"):
            add(rows, plans["stale"], warn, 0.45)
        kick, strike, block, guard = plans.get("kick"), plans.get("strike"), plans.get("block"), plans.get("guard")
        showing = (mach or {}).get("state")
        if block:
            add(rows, f"OWN GOAL in {block['enters_in_s']:.1f} s: " + ("no block point" if block["spare_s"] is None else _spare(block)),
                (0, 0, 255), 0.5, 2)
        if showing in ("BLOCK", "GUARD"):
            plan = block if showing == "BLOCK" else guard
            for k, step in enumerate((plan or {}).get("steps") or [], 1):
                add(rows, f"{k}. {step}", (200, 200, 200), 0.44, 1, 18)
        elif kick and kick.get("feasible"):
            add(rows, f"KICK, {kick['paw']} paw: contact in {kick['contact_in_s']:.1f} s"
                + (f" (saves {kick['saves_s']:.1f} s)" if kick["saves_s"] > 0.05 else ""), (0, 255, 0), 0.5, 1, 20)
            if detail:      # the explanations may have been trimmed off a state that came over the network
                if kick.get("timeline"):
                    add(rows, " + ".join(f"{t['what']} {t['s']:.1f}" for t in kick["timeline"]) + " s", grey, 0.44, 1, 18)
                if kick.get("limit"):
                    add(rows, f"limited by the {kick['limit']}"
                        + (f"; {kick['other_paw']['paw']} paw would take {kick['other_paw']['contact_in_s']:.1f} s" if kick.get("other_paw") else ""),
                        grey, 0.44, 1, 18, 21)
                if kick.get("steps"):
                    rows.append(("ROUTE", grey, 0.42, 1, 18))
                for k, step in enumerate(kick.get("steps") or [], 1):
                    add(rows, f"{k}. {step}", (200, 200, 200), 0.44, 1, 18)
                if kick.get("why"):
                    add(rows, kick["why"], grey, 0.42, 1, 17, 21)
            for text in kick.get("warnings") or []:
                add(rows, "! " + text, warn, 0.44)
        elif kick:
            add(rows, f"no kick: {kick.get('note', 'not possible right now')}", (170, 170, 170), 0.47)
        if detail and strike and strike.get("feasible"):
            add(rows, f"run-through instead: line up in {strike['travel_s']:.1f} s, contact in {strike['contact_in_s']:.1f} s", grey, 0.42)
        rel = (state.get("relative") or {}).get(name, {})
        if detail and "ball" in rel:
            b = rel["ball"]
            add(rows, f"ball {b['distance'] * 100:.0f} cm away, {abs(math.degrees(b['bearing'])):.0f} deg "
                f"{'left' if b['bearing'] >= 0 else 'right'}", grey, 0.42)
        ap = (state.get("approach") or {}).get(name)
        if ap:
            phase_col = {"APPROACH": (0, 200, 255), "ALIGN": (0, 255, 255), "RUN": (0, 220, 0), "BRAKE": (170, 170, 170)}
            rows.append((f"RUN-UP LOGIC: {ap['phase']}", phase_col.get(ap["phase"], white), 0.5, 2, 20))
            cmd = ap["cmd"]
            add(rows, f"cmd  fwd {cmd['vx']:+.2f}  left {cmd['vy']:+.2f} m/s  turn {cmd['wz']:+.2f}", white, 0.44)
            if detail:
                for k, step in enumerate(((ap.get("plan") or {}).get("steps") or [])[:5], 1):
                    add(rows, f"{k}. {step}", (200, 200, 200), 0.42, 1, 17)
        add(rows, f"age {r['age_s'] * 1000:.0f} ms", grey, 0.42, 1, 20, 26)
        out.append(rows)
    return out


def _draw_rows(img, rows, x, y):
    for text, colour, scale, thick, gap in rows:
        _text(img, text, (x, y), colour, scale, thick)
        y += gap
    return y


def render_panel(state: dict | None, size=(PANEL_W, 800), stale_s: float = 0.0, zoom: str = "field",
                 detail: bool = True) -> np.ndarray:
    """The readouts as one tall column (the standalone dashboard's right-hand side). If they do not
    fit, the least important sections (the robots' details) run off the bottom: use the strip layout."""
    w, h = size
    img = np.full((h, w, 3), (18, 18, 18), np.uint8)
    cv2.line(img, (0, 0), (0, h), (70, 70, 70), 1)
    _text(img, "Pupper FC field view", (16, 30), (255, 255, 255), 0.7, 2)
    y = 56
    for rows in _sections(state, stale_s, detail, w - 30):
        y = _draw_rows(img, rows, 16, y) + 4
    cv2.rectangle(img, (0, h - 28), (w, h), (18, 18, 18), -1)
    _text(img, f"z = zoom ({'fit' if zoom == 'fit' else 'whole field'})   i = {'less' if detail else 'more'} detail   q = quit",
          (16, h - 10), (150, 150, 150), 0.42)
    return img


def render_strip(state: dict | None, size=(1400, 250), stale_s: float = 0.0, detail: bool = True) -> np.ndarray:
    """The same readouts laid out as columns, to sit underneath the two views."""
    w, h = size
    columns = max(len((state or {}).get("robots") or {}) + 2, 4)
    col_w = w // columns
    sections = _sections(state, stale_s, detail, col_w - 30)
    h = max(h, 30 + max(sum(row[4] for row in rows) for rows in sections))  # grow to fit the longest column
    img = np.full((h, w, 3), (18, 18, 18), np.uint8)
    cv2.line(img, (0, 0), (w, 0), (70, 70, 70), 1)
    for k, rows in enumerate(sections):
        if k:
            cv2.line(img, (k * col_w, 10), (k * col_w, h - 10), (50, 50, 50), 1)
        _draw_rows(img, rows, k * col_w + 16, 30)
    return img


def render(state: dict | None, size=(1280, 800), zoom: str = "field", stale_s: float = 0.0,
           detail: bool = True) -> np.ndarray:
    """The standalone dashboard: field view on top, readouts in columns underneath (they need the width).
    The picture is at least `size`; it grows downward when the readouts need more room."""
    w, h = size
    field = render_field(state, field_view_size(size), zoom, stale_s)
    strip = render_strip(state, (w, max(h - field.shape[0], 120)), stale_s, detail)
    return np.vstack([field, strip])


def field_view_size(size) -> tuple[int, int]:
    """(width, height) of the field view at the top of render()'s picture: for mapping mouse clicks."""
    return size[0], max(int(size[1] * 0.6), 400)


def render_combined(camera: np.ndarray, state: dict | None, width: int = 1400, zoom: str = "field",
                    stale_s: float = 0.0, camera_share: float = 0.62, detail: bool = True):
    """One picture for the live program: camera feed (left) beside the field view (right),
    readouts underneath. Returns (picture, camera_scale, (camera_w, camera_h)); the camera
    sits at the top-left corner, so a window pixel (x, y) inside it is camera pixel
    (x / camera_scale, y / camera_scale)."""
    cam_w = int(width * camera_share)
    k = cam_w / camera.shape[1]
    cam_h = int(round(camera.shape[0] * k))
    cam = cv2.resize(camera, (cam_w, cam_h), interpolation=cv2.INTER_AREA)
    top = np.hstack([cam, render_field(state, (width - cam_w, cam_h), zoom, stale_s)])
    return np.vstack([top, render_strip(state, (width, 250), stale_s, detail)]), k, (cam_w, cam_h)
