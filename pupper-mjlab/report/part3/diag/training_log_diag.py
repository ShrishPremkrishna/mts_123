import re, sys, json, glob
from pathlib import Path
import numpy as np
R = Path.home() / "runs/opt-lab-2"
KEYS = ["Mean reward", "Mean episode length", "Mean action std", "Mean entropy loss", "Mean value loss",
        "Episode_Termination/fell_over", "Episode_Termination/time_out", "Metrics/twist/error_vel_xy",
        "Metrics/twist/error_vel_yaw", "Episode_Reward/track_linear_velocity", "Episode_Reward/gait_tracking",
        "Episode_Reward/upright", "Episode_Reward/ang_vel_xy_l2", "Episode_Reward/action_rate_l2",
        "Episode_Reward/knee_ground_contact", "Episode_Reward/stand_still_pose", "Episode_Reward/track_yaw_velocity",
        "Episode_Reward/lin_vel_z_l2", "Episode_Reward/abduction_angle", "Episode_Reward/roll_asymmetry",
        "Episode_Reward/stand_still_joint_velocity"]
def parse(log):
    t = Path(log).read_text(errors="ignore")
    blocks = re.split(r"Learning iteration (\d+)/\d+", t)
    rows = {}
    for i in range(1, len(blocks) - 1, 2):
        it = int(blocks[i]); b = blocks[i + 1]
        r = {}
        for k in KEYS:
            m = re.search(re.escape(k) + r":\s*([-0-9.eE+]+)", b)
            if m: r[k] = float(m.group(1))
        rows[it] = r
    return rows
def lr(run_dir):
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError:
        return None
    ev = sorted(glob.glob(str(run_dir / "events.out.tfevents*")))
    if not ev: return None
    ea = EventAccumulator(ev[-1], size_guidance={"scalars": 0}); ea.Reload()
    tags = [t for t in ea.Tags()["scalars"] if "learning_rate" in t.lower()]
    if not tags: return None
    v = [(s.step, s.value) for s in ea.Scalars(tags[0])]
    return v
def win(rows, lo, hi):
    sel = [r for it, r in rows.items() if lo <= it <= hi]
    return {k: round(float(np.mean([r[k] for r in sel if k in r])), 4) for k in KEYS if any(k in r for r in sel)}, len(sel)
out = {}
base = parse(R / "rp-p2-s42-r2-cont-1500.log")
bmax = max(base); out["P (W3 warm start, last 100 its)"] = win(base, bmax - 99, bmax)
for arm in ("bumpyft", "flatctrl"):
    for leg in (1, 2, 3):
        d = R / f"rp-p3-final-{arm}-kpwide-leg{leg}"
        lg = d.with_name(d.name + ".log")
        if not lg.exists(): continue
        rows = parse(lg)
        if not rows: continue
        lo, hi = min(rows), max(rows)
        out[f"{arm} leg{leg} first 50 ({lo}-)"] = win(rows, lo, lo + 49)
        out[f"{arm} leg{leg} last 100 (-{hi})"] = win(rows, hi - 99, hi)
        l = lr(d)
        if l:
            vals = np.array([v for _, v in l])
            out[f"{arm} leg{leg} lr"] = dict(first=float(vals[0]), last=float(vals[-1]), min=float(vals.min()),
                                              max=float(vals.max()), median=float(np.median(vals)))
        snap = d / "randomization_snapshot.npz"
        if snap.exists():
            z = np.load(snap); kp = z["pd_stiffness"] / 5.5; kd = z["pd_damping"] / 0.25
            out[f"{arm} leg{leg} realized"] = dict(kp=[round(float(kp.min()), 3), round(float(kp.mean()), 3), round(float(kp.max()), 3)],
                                                   kd=[round(float(kd.min()), 3), round(float(kd.mean()), 3), round(float(kd.max()), 3)],
                                                   frac_meankp_ge1=round(float((kp.mean(1) >= 1).mean()), 3))
lp = lr(R / "rp-p2-s42-r2-cont-1500")
if lp:
    vals = np.array([v for _, v in lp[-200:]]); out["P lr (last 200)"] = dict(median=float(np.median(vals)), min=float(vals.min()), max=float(vals.max()))
json.dump(out, open(sys.argv[1], "w"), indent=1)
for k, v in out.items():
    print(k, json.dumps(v if not isinstance(v, tuple) else {**v[0], "n": v[1]}))
