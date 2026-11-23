#!/usr/bin/env python3
"""Ball-kick trial helper (2026-11-18): call API tools and fit the ball calibration (stage R3).

    ballkick-calib.py call TOOL ['{"side": "right"}']   any tool, with the current command epoch
    ballkick-calib.py measure TRUE_X TRUE_Y [N]          N locate_ball calls (default 3), rows to calib.csv
    ballkick-calib.py fit [--write]                      fit range_scale, mount.x, y_bias, ball_z from calib.csv

TRUE_X/TRUE_Y: the ball centre measured with a ruler from base_link (x forward, y left, m).
locate_ball does not move the robot. fit --write keeps a backup of the old file and writes
calibrated: true; run it only after the operator accepts the printed fit.
"""
import csv, json, math, os, shutil, statistics, sys, time, urllib.error, urllib.request
from pathlib import Path

API = "http://127.0.0.1:18090"
RUN = Path("/home/pi/pupper-foundation-session-20261029/supervised-console-runs/20261118-ballseek")
CSV = RUN / "calib.csv"
CONFIG = Path(os.environ.get("PUPPER_BALL_KICK_CONFIG", "~/.config/gemini-pupper/ball_kick.yaml")).expanduser()
FIELDS = ["time", "true_x", "true_y", "x", "y", "z", "distance", "bearing_deg", "method", "rho_deg", "fixes", "frames"]


def call(tool, args=None):
    with urllib.request.urlopen(API + "/health", timeout=5) as r:
        epoch = json.load(r).get("command_epoch")
    req = urllib.request.Request(API + "/tools/" + tool, data=json.dumps(args or {}).encode(), method="POST",
                                 headers={"Content-Type": "application/json", "X-Pupper-Command-Epoch": epoch or ""})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return json.loads(e.read() or b"{}")


def current():
    try:
        import yaml
        return yaml.safe_load(CONFIG.read_text()) or {} if CONFIG.is_file() else {}
    except Exception as e:
        sys.exit(f"cannot read {CONFIG}: {e}")


def measure(tx, ty, n):
    new = not CSV.exists()
    with CSV.open("a", newline="") as f:
        w = csv.DictWriter(f, FIELDS)
        if new:
            w.writeheader()
        for i in range(n):
            res = call("locate_ball")
            ball = (res.get("details") or {}).get("ball") or {}
            print(f"{i + 1}/{n}: {res.get('message')}")
            if ball.get("seen"):
                w.writerow({"time": time.strftime("%H:%M:%S"), "true_x": tx, "true_y": ty,
                            **{k: ball.get(k) for k in FIELDS[3:]}})
            if i + 1 < n and sys.stdin.isatty():
                input("  turn the ball (gore axis vertical / horizontal / tilted), then press Enter ")


def fit(write):
    rows = [r for r in csv.DictReader(CSV.open()) if r["method"] == "fit"]
    if len(rows) < 4:
        sys.exit("need at least 4 'fit' rows")
    cfg = current()
    mount, vision = cfg.get("mount", {}), cfg.get("vision", {})
    s0, mx0, mz0 = mount.get("range_scale", 1.0), mount.get("x", 0.15), mount.get("z", 0.03)
    yb0 = vision.get("y_bias", 0.0)
    f = lambda r, k: float(r[k])
    near = [r for r in rows if math.hypot(f(r, "true_x"), f(r, "true_y")) <= 0.85]
    if len({round(f(r, "true_x"), 2) for r in near}) < 2:
        sys.exit("need near rows (<= 0.85 m) at two or more distances, e.g. 0.50 and 0.76 m")
    xs, xt = [f(r, "x") for r in near], [f(r, "true_x") for r in near]
    mx, mt = statistics.fmean(xs), statistics.fmean(xt)
    beta = sum((a - mx) * (b - mt) for a, b in zip(xs, xt)) / sum((a - mx) ** 2 for a in xs)
    alpha = mt - beta * mx
    scale, mount_x = s0 * beta, alpha + beta * mx0
    y_bias = statistics.fmean(beta * (f(r, "y") + yb0) - f(r, "true_y") for r in rows)
    zs = [mz0 + beta * (f(r, "z") - mz0) for r in rows]
    ball_z, z_sd = statistics.fmean(zs), (statistics.stdev(zs) if len(zs) > 1 else 0.0)
    print(f"fit from {len(near)} near rows and {len(rows)} rows: range_scale {scale:.4f} (was {s0}), "
          f"mount.x {mount_x:.4f} (was {mx0}), y_bias {y_bias:+.4f} (was {yb0}), ball_z {ball_z:+.4f} (sd {z_sd:.4f})")
    print("residuals after the fit (true - corrected), m:")
    for r in rows:
        cx = alpha + beta * f(r, "x")
        cy = beta * (f(r, "y") + yb0) - y_bias
        print(f"  true ({f(r, 'true_x'):.3f}, {f(r, 'true_y'):+.3f})  dx {f(r, 'true_x') - cx:+.4f}  dy {f(r, 'true_y') - cy:+.4f}")
    new = dict(cfg)
    new["calibrated"] = True
    new.setdefault("mount", {}).update(range_scale=round(scale, 4), x=round(mount_x, 4))
    new.setdefault("vision", {}).update(y_bias=round(y_bias, 4), ball_z=round(ball_z, 4),
                                       z_tol=round(max(0.02, 3 * z_sd), 3))
    import yaml
    text = yaml.safe_dump(new, sort_keys=False)
    print("\nproposed", CONFIG, ":\n" + text)
    if write:
        if CONFIG.exists():
            shutil.copy2(CONFIG, CONFIG.with_suffix(".yaml.bak-" + time.strftime("%Y%m%d-%H%M%S")))
        CONFIG.write_text("# kick_ball calibration, fitted by ballkick-calib.py " + time.strftime("%Y-%m-%d %H:%M") + "\n" + text)
        print("written")


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == "call":
        print(json.dumps(call(a[1], json.loads(a[2]) if len(a) > 2 else {}), indent=2))
    elif a[0] == "measure":
        measure(float(a[1]), float(a[2]), int(a[3]) if len(a) > 3 else 3)
    elif a[0] == "fit":
        fit("--write" in a)
    else:
        sys.exit(__doc__)
