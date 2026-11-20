#!/usr/bin/env python3
"""Generate printable AprilTag (36h11) pages for every tag in the config, each labelled
with where it goes, plus one layout sheet showing the whole field from above.

  python scripts/make_tags.py            # PNG pages into out/tags/, layout into tags/tag_layout.png
  python scripts/make_tags.py --pdf      # also tags/pupper_fc_tags.pdf (needs Pillow, see below)

Each tag page is US Letter at 300 dpi; print at 100% / "actual size" on matte paper
(glossy paper glares under overhead lights and kills detection).
Floor tags: tape the whole sheet down, any rotation, tag centre on the spot the page names.
Robot tags: cut out at the corner marks (keeps the white border and the small FRONT
note), mount flat with the FRONT edge toward the robot's nose.

The PDF step uses Pillow, which the project does not otherwise need:
  uv run --with pillow --with-requirements requirements.txt python scripts/make_tags.py --pdf
"""
import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.config import ROOT, load_config  # noqa: E402
from fieldcam.detect import TAG_DICT  # noqa: E402

DPI = 300
CUT_BORDER_M = 0.014  # white border left around a robot tag when it is cut out
FONT = cv2.FONT_HERSHEY_SIMPLEX
ROLE_TEXT = {
    "west_south": "CORNER: west goal end, south side",
    "east_south": "CORNER: east goal end, south side",
    "east_north": "CORNER: east goal end, north side",
    "west_north": "CORNER: west goal end, north side",
}


def placements(cfg: dict) -> dict:
    """Where each floor tag goes: {id: {"where": words, "xy": field position of the tag centre}}.

    Corner tags come from tags.corner_ids and sit on the field corners (moved inward by
    tags.corner_inset). Any other floor tag goes on the halfway line, at the side its
    configured y points to.
    """
    f, t = cfg["field"], cfg["tags"]
    hl, hw = f["length"] / 2, f["width"] / 2
    ix, iy = t["corner_inset"]
    spot = {"west_south": (-hl + ix, -hw + iy), "east_south": (hl - ix, -hw + iy),
            "east_north": (hl - ix, hw - iy), "west_north": (-hl + ix, hw - iy)}
    out = {tid: {"where": ROLE_TEXT[role], "xy": spot[role]} for role, tid in t["corner_ids"].items()}
    for tid, xy in t["field"].items():
        if tid not in out:
            north = xy[1] >= 0
            out[tid] = {"where": f"HALFWAY LINE: {'north' if north else 'south'} side",
                        "xy": (0.0, (hw - iy) if north else (-hw + iy))}
    return out


def tag_image(tag_id: int, side_px: int) -> np.ndarray:
    side_px -= side_px % 8  # whole pixels per tag cell
    return cv2.resize(cv2.aruco.generateImageMarker(TAG_DICT, tag_id, 8), (side_px, side_px), interpolation=cv2.INTER_NEAREST)


def draw_minimap(img: np.ndarray, x: int, y: int, w: int, h: int, cfg: dict, spots: dict, highlight: int):
    """A small top-down field (grayscale) with every floor tag's spot; the highlighted one is filled."""
    f = cfg["field"]
    k = min((w - 90) / f["length"], (h - 60) / f["width"])
    cx, cy = x + w // 2, y + h // 2
    P = lambda fx, fy: (int(round(cx + fx * k)), int(round(cy - fy * k)))  # noqa: E731
    hl, hw, hg = f["length"] / 2, f["width"] / 2, f["goal_width"] / 2
    cv2.rectangle(img, P(-hl, hw), P(hl, -hw), 0, 3)
    cv2.line(img, P(0, hw), P(0, -hw), 0, 1)
    for sign, letter in ((-1, "W"), (1, "E")):
        cv2.rectangle(img, P(sign * hl, hg), P(sign * (hl + 0.18), -hg), 0, 3)
        gx, gy = P(sign * (hl + 0.18), 0)
        cv2.putText(img, letter, (gx + (8 if sign > 0 else -34), gy + 12), FONT, 1.0, 0, 3)
    cv2.putText(img, "N", (cx - 12, P(0, hw)[1] - 8), FONT, 0.8, 0, 2)
    cv2.putText(img, "S", (cx - 12, P(0, -hw)[1] + 30), FONT, 0.8, 0, 2)
    for tid, s in spots.items():
        px, py = P(*s["xy"])
        if tid == highlight:
            cv2.rectangle(img, (px - 13, py - 13), (px + 13, py + 13), 0, -1)
            cv2.circle(img, (px, py), 30, 0, 3)
        else:
            cv2.rectangle(img, (px - 8, py - 8), (px + 8, py + 8), 255, -1)
            cv2.rectangle(img, (px - 8, py - 8), (px + 8, py + 8), 0, 2)


def page(tag_id: int, size_m: float, label: str, front: str | None = None, where: str | None = None,
         cfg: dict | None = None, spots: dict | None = None) -> np.ndarray:
    """One US-Letter page. `front`: robot tags only, a small note that stays on the tag after cutting.
    `where` (+ cfg, spots): floor tags, the words and a small map saying where the tag goes."""
    img = np.full((int(11 * DPI), int(8.5 * DPI)), 255, np.uint8)
    side = int(round(size_m / 0.0254 * DPI))
    side -= side % 8
    y0, x0 = (img.shape[0] - side) // 2, (img.shape[1] - side) // 2
    img[y0:y0 + side, x0:x0 + side] = tag_image(tag_id, side)
    cell = side // 8  # the tag needs this much clear white on every side
    # Labels and the map stay clear of that white zone and of the printer's margins.
    top = max(y0 - side // 4 - 30, 170)
    cv2.putText(img, f"{label}   id {tag_id}   {size_m * 100:.0f} cm", (x0, top), FONT, 1.6, 0, 4)
    if where:
        cv2.putText(img, where, (x0, top + 75), FONT, 1.5, 0, 4)
        map_top = y0 + side + cell + 20
        map_h = img.shape[0] - 80 - map_top
        if cfg is not None and spots is not None and map_h > 150:
            draw_minimap(img, x0, map_top, 560, map_h, cfg, spots, tag_id)
            cv2.putText(img, "PUT THIS TAG HERE (seen from above).", (x0 + 600, map_top + 70), FONT, 1.2, 0, 3)
            cv2.putText(img, "Tag centre on the spot, any rotation.", (x0 + 600, map_top + 125), FONT, 1.2, 0, 3)
            cv2.putText(img, "E = east goal, the one the striker attacks.", (x0 + 600, map_top + 180), FONT, 1.0, 0, 2)
    if front:
        # Robot tags get cut out, so the front marker must sit inside the cut:
        # beyond the one-cell white quiet zone, inside corner cut marks at CUT_BORDER.
        b = int(round(CUT_BORDER_M / 0.0254 * DPI))
        assert b > cell + 45, "tag too large for a front marker inside the cut border"
        (tw, _), _ = cv2.getTextSize(front, FONT, 1.0, 2)
        cv2.putText(img, front, (x0 + (side - tw) // 2, y0 - cell - 8), FONT, 1.0, 0, 2)
        for cx, sx in ((x0 - b, 1), (x0 + side + b, -1)):
            for cy, sy in ((y0 - b, 1), (y0 + side + b, -1)):
                cv2.line(img, (cx, cy), (cx + sx * 60, cy), 0, 2)
                cv2.line(img, (cx, cy), (cx, cy + sy * 60), 0, 2)
    return img


def layout_sheet(cfg: dict, floor_size: float, robot_size: float) -> np.ndarray:
    """A landscape US-Letter sheet: the field from above with every tag where it goes."""
    W, H = int(11 * DPI), int(8.5 * DPI)
    img = np.full((H, W, 3), 255, np.uint8)
    f, spots = cfg["field"], placements(cfg)
    hl, hw, hg = f["length"] / 2, f["width"] / 2, f["goal_width"] / 2
    k = min((W - 1100) / f["length"], (H - 1050) / f["width"])       # pixels per metre
    cx, cy = W // 2, 330 + int(hw * k) + 60
    P = lambda x, y: (int(round(cx + x * k)), int(round(cy - y * k)))  # noqa: E731
    black, grey, goal_col, pink = (0, 0, 0), (110, 110, 110), (0, 140, 255), (190, 105, 255)

    def text(s, org, scale=1.2, colour=black, thick=3):
        cv2.putText(img, s, (int(org[0]), int(org[1])), FONT, scale, colour, thick, cv2.LINE_AA)

    text("Pupper FC: where the tags go (seen from above)", (120, 130), 2.2, black, 5)
    text(f"Field {f['length']:.2f} x {f['width']:.2f} m. Floor tags {floor_size * 100:.0f} cm, robot tags {robot_size * 100:.0f} cm. "
         "Drawn to scale.", (120, 205), 1.2, grey, 2)

    # Field, halfway line, goals, names of the four sides.
    cv2.rectangle(img, P(-hl, hw), P(hl, -hw), (225, 240, 225), -1)
    cv2.rectangle(img, P(-hl, hw), P(hl, -hw), black, 6)
    cv2.line(img, P(0, hw), P(0, -hw), black, 3)
    cv2.circle(img, P(0, 0), 10, black, -1)
    for sign, name in ((-1, "WEST GOAL"), (1, "EAST GOAL")):
        cv2.rectangle(img, P(sign * hl, hg), P(sign * (hl + 0.22), -hg), goal_col, 8)
        gx = P(sign * (hl + 0.22), 0)[0]
        text(name, (gx + 20 if sign > 0 else gx - 330, cy - 20), 1.5, goal_col, 4)
        text("(+x)" if sign > 0 else "(-x)", (gx + 20 if sign > 0 else gx - 330, cy + 40), 1.2, goal_col, 3)
    quarter = P(-hl / 2, 0)[0] - 220   # between the corner tag and the halfway tag, clear of both labels
    text("NORTH side (+y)", (quarter, P(0, hw)[1] - 60), 1.5, grey, 3)
    text("SOUTH side (-y)", (quarter, P(0, -hw)[1] + 110), 1.5, grey, 3)

    # Axes at the centre.
    cv2.arrowedLine(img, P(0, 0), P(0.45, 0), grey, 4, cv2.LINE_AA, tipLength=0.15)
    cv2.arrowedLine(img, P(0, 0), P(0, 0.45), grey, 4, cv2.LINE_AA, tipLength=0.15)
    text("+x", (P(0.47, 0)[0], cy + 14), 1.1, grey, 2)
    text("+y", (cx - 30, P(0, 0.47)[1] - 10), 1.1, grey, 2)

    def paste_tag(tag_id, centre_px, size_m, angle_deg=0.0):
        side = max(int(round(size_m * k)), 16)
        tag = tag_image(tag_id, side)
        pad = max(tag.shape[0] // 8, 2)
        sheet = cv2.copyMakeBorder(tag, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)  # the white border
        if angle_deg:
            n = sheet.shape[0]
            M = cv2.getRotationMatrix2D((n / 2, n / 2), angle_deg, 1.0)
            sheet = cv2.warpAffine(sheet, M, (n, n), borderValue=255, flags=cv2.INTER_NEAREST)
        n = sheet.shape[0]
        x, y = centre_px[0] - n // 2, centre_px[1] - n // 2
        img[y:y + n, x:x + n] = cv2.cvtColor(sheet, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(img, (x, y), (x + n, y + n), grey, 2)
        return n

    # Floor tags, with what each one is.
    for tid, s in sorted(spots.items()):
        c = P(*s["xy"])
        n = paste_tag(tid, c, floor_size)
        east, north = s["xy"][0] > 0.01, s["xy"][1] > 0
        label = f"TAG {tid}"
        role = "corner" if s["where"].startswith("CORNER") else "halfway line (optional)"
        (tw, _), _ = cv2.getTextSize(label, FONT, 1.5, 4)
        lx = c[0] + n // 2 + 16 if east or abs(s["xy"][0]) < 0.01 else c[0] - n // 2 - 16 - max(tw, 150)
        ly = c[1] - n // 2 - 50 if north else c[1] + n // 2 + 55
        text(label, (lx, ly), 1.5, black, 4)
        text(role, (lx, ly + 42), 0.95, grey, 2)

    # Robots, each with its tag on its back, and the ball.
    for name, rc, (x, y, yaw) in ((n, cfg["robots"][n], pose) for n, pose in
                                  (("striker", (-hl * 0.45, -hw * 0.35, 0.35)), ("keeper", (hl * 0.50, hw * 0.45, math.pi)))
                                  if n in cfg["robots"]):
        c, s = math.cos(yaw), math.sin(yaw)
        body = [(x + c * a - s * b, y + s * a + c * b) for a, b in ((0.17, 0.10), (0.17, -0.10), (-0.17, -0.10), (-0.17, 0.10))]
        cv2.polylines(img, [np.int32([P(*q) for q in body])], True, black, 5, cv2.LINE_AA)
        paste_tag(rc["tag_id"], P(x, y), robot_size, math.degrees(yaw) - 90.0)   # tag top (FRONT) toward the nose
        cv2.arrowedLine(img, P(x + c * 0.19, y + s * 0.19), P(x + c * 0.42, y + s * 0.42), black, 5, cv2.LINE_AA, tipLength=0.3)
        px, py = P(x, y)
        ty = py + int(0.33 * k) if y < 0 else py - int(0.33 * k) - 50   # caption below a robot in the south half, above in the north
        text(f"{name.upper()}: TAG {rc['tag_id']} on its back", (px - 330, ty), 1.2, black, 3)
        text(f"FRONT edge of the tag toward the nose; attacks the {rc['attacks'].upper()} goal", (px - 330, ty + 42), 0.95, grey, 2)
    bx, by = P(hl * 0.30, -hw * 0.30)
    cv2.circle(img, (bx, by), max(int(cfg["ball"]["radius"] * k), 8), pink, -1, cv2.LINE_AA)
    cv2.circle(img, (bx, by), max(int(cfg["ball"]["radius"] * k), 8), black, 2, cv2.LINE_AA)
    text("ball (no tag: found by colour)", (bx + 60, by + 14), 0.95, grey, 2)

    r = cfg["tags"]["corner_ids"]
    lines = [
        f"CORNER TAGS {r['west_south']}, {r['east_south']}, {r['east_north']}, {r['west_north']}: centre of the tag on the field corner. Going round the field "
        f"counter-clockwise they read {r['west_south']} > {r['east_south']} > {r['east_north']} > {r['west_north']}. Any rotation.",
        f"The EAST goal is between tags {r['east_south']} and {r['east_north']}; the WEST goal between tags {r['west_north']} and {r['west_south']}. "
        "East/west/north/south are only names: pick which goal is EAST once, the rest follows.",
        "HALFWAY-LINE TAGS: optional. They do not change the field; they help locate the camera more accurately.",
        "ALL tags must be completely visible to the overhead camera when calibrating: whole black square plus white border, nothing on top.",
        "ROBOT TAGS: cut at the corner marks, glue to stiff card, mount flat on the robot's back with the FRONT edge toward the nose.",
    ]
    for i, ln in enumerate(lines):
        text(ln, (120, H - 330 + 58 * i), 1.0, black, 2)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(ROOT / "out" / "tags"), help="folder for the PNG pages")
    ap.add_argument("--field-size", type=float, default=None,
                    help="floor tag side, m (default: config; 0.17 is the largest that keeps a white border on US Letter)")
    ap.add_argument("--robot-size", type=float, default=None, help="robot tag side, m (default: config)")
    ap.add_argument("--pdf", action="store_true", help="also write tags/pupper_fc_tags.pdf (needs Pillow)")
    args = ap.parse_args()
    cfg = load_config()
    args.field_size = args.field_size or cfg["tags"]["floor_size"]
    args.robot_size = args.robot_size or cfg["tags"]["robot_size"]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    spots = placements(cfg)

    pages = []
    layout = layout_sheet(cfg, args.field_size, args.robot_size)
    (ROOT / "tags").mkdir(exist_ok=True)
    cv2.imwrite(str(ROOT / "tags" / "tag_layout.png"), cv2.resize(layout, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
    cv2.imwrite(str(out / "00_layout.png"), layout)
    pages.append(out / "00_layout.png")
    for tid in sorted(cfg["tags"]["field"]):
        path = out / f"floor_{tid:02d}.png"
        cv2.imwrite(str(path), page(tid, args.field_size, "FLOOR TAG", where=spots[tid]["where"], cfg=cfg, spots=spots))
        pages.append(path)
    for name, rc in cfg["robots"].items():
        path = out / f"robot_{name}_{rc['tag_id']:02d}.png"
        cv2.imwrite(str(path), page(rc["tag_id"], args.robot_size, f"{name} (attacks {rc['attacks']}): cut at the corner marks",
                                    front=f"^ FRONT   {name}   id {rc['tag_id']} ^"))
        pages.append(path)
    print(f"wrote {len(pages)} pages to {out}/ and the layout picture to tags/tag_layout.png")

    if args.pdf:
        try:
            from PIL import Image
        except ImportError:
            raise SystemExit("--pdf needs Pillow. Run:\n  uv run --with pillow --with-requirements requirements.txt "
                             "python scripts/make_tags.py --pdf") from None
        images = [Image.open(p).convert("RGB") for p in pages]
        pdf = ROOT / "tags" / "pupper_fc_tags.pdf"
        images[0].save(pdf, save_all=True, append_images=images[1:], resolution=float(DPI))
        print(f"wrote {pdf.relative_to(ROOT)}: layout sheet + {len(pages) - 1} tag pages")


if __name__ == "__main__":
    main()
