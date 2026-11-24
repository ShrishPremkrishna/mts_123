#!/usr/bin/env python3
"""MuJoCo check of the gentle lie-down (stack_overlay.py --liedown, fcctl `down`). Laptop only, not staged.

  cd ~/workspace/cs123 && FORCE_CPU=1 .venv/bin/python final_project/fc/robot/sim/liedown_sim.py [--json OUT]

Model: pupper-mjlab Pupper v3 with full-hull ground contact (foundation/scripts/check_spider_handoff.make_model,
520 Hz = the controller manager rate), position servos tau = kp (q* - q) - kd qd, |tau| <= 3 Nm. Sequence,
with the gains the real controllers use:
  1. stand: PD on the stance (kp 5.5 / kd 0.25) for 2 s (stands in for the walker at zero command),
  2. lie-down controller activated: linear blend from the measured pose to LIE_POSE over LIEDOWN_INIT_S at
     init_kps/init_kds 7.5/0.25 (neural_controller init phase), then hold at the policy JSON kp/kd 5.0/0.25,
  3. deactivated: kp 0, kd = estop_kd 0.1 (neural_controller::on_deactivate), 2 s,
  4. re-stand: blend to the stance over 2 s at 7.5/0.25 (the walker's init phase; no policy afterwards).
Baseline: R3 from standing (step 3 straight after step 1).
Pass: belly resting on the floor after step 2, max tilt < 10 deg in step 2, base drop < 2 cm in step 3,
re-stand tilt < 10 deg, for every case (gain, friction, mass, initial pose and timing variations).
Not modelled: motor latency/backlash, the walker's own stepping while standing, IMU, real knee stops.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from pathlib import Path

import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]                      # ~/workspace/cs123
sys.path.insert(0, str(HERE.parent))
import stack_overlay as so  # noqa: E402

STANCE = np.array([0.26, 0.0, -0.52, -0.26, 0.0, 0.52] * 2)
LIE = np.array(so.LIE_POSE)
INIT_KP, INIT_KD, HOLD_KP, HOLD_KD, STAND_KP, STAND_KD, ESTOP_KD = 7.5, 0.25, 5.0, 0.25, 5.5, 0.25, 0.1


def load_model():
    spec = importlib.util.spec_from_file_location("hand", ROOT / "foundation/scripts/check_spider_handoff.py")
    hand = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hand)
    m, _ = hand.make_model("full_hulls")
    return m


class Sim:
    def __init__(self):
        m = self.m = load_model()
        self.qadr = m.jnt_qposadr[[m.joint(n).id for n in so.JOINT_NAMES]]
        self.ctl = np.array([int(np.flatnonzero(m.actuator_trnid[:, 0] == m.joint(n).id)[0]) for n in so.JOINT_NAMES])
        self.base = m.body("base_link").id
        rj = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE][0]
        self.rq, self.rv = m.jnt_qposadr[rj], m.jnt_dofadr[rj]
        self.hull, self.terrain = m.geom("base_link_hull").id, m.geom("terrain").id
        self.fric0, self.mass0 = m.geom_friction[:, 0].copy(), float(m.body_mass[self.base])
        self.dt = m.opt.timestep

    def pd(self, kp, kd):
        m = self.m
        m.actuator_gainprm[self.ctl, 0] = kp
        m.actuator_biasprm[self.ctl, 1] = -kp
        m.actuator_biasprm[self.ctl, 2] = -kd

    def tilt(self, d):
        q = d.qpos[self.rq + 3:self.rq + 7]
        return float(np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1))))

    def belly(self, d):
        return any({int(c.geom1), int(c.geom2)} == {self.hull, self.terrain} for c in d.contact[:d.ncon])

    def run(self, T=so.LIEDOWN_INIT_S, hold=1.0, fric=1.0, mass=0.0, kps=1.0, off=None, pitch0=0.0, r3=False):
        m = self.m
        m.geom_friction[:, 0] = self.fric0 * fric
        m.body_mass[self.base] = self.mass0 + mass
        try:
            d = mujoco.MjData(m)
            d.qpos[self.rq:self.rq + 3] = [0, 0, 0.18]
            d.qpos[self.rq + 3:self.rq + 7] = [np.cos(pitch0 / 2), 0, np.sin(pitch0 / 2), 0]
            st = STANCE + (0 if off is None else off)
            d.qpos[self.qadr] = st
            mujoco.mj_forward(m, d)
            step = lambda n: [mujoco.mj_step(m, d) for _ in range(int(n / self.dt))]  # noqa: E731
            self.pd(STAND_KP * kps, STAND_KD)
            d.ctrl[self.ctl] = st
            step(2.0)
            out = {"z_stand_cm": round(100 * d.xpos[self.base][2], 1)}
            if not r3:
                q0 = d.qpos[self.qadr].copy()
                self.pd(INIT_KP * kps, INIT_KD)
                tl, vz, fmax = [], [], 0.0
                for k in range(int((T + hold) / self.dt)):
                    a = min(1.0, k * self.dt / T)
                    if k * self.dt >= T:
                        self.pd(HOLD_KP * kps, HOLD_KD)
                    d.ctrl[self.ctl] = (1 - a) * q0 + a * LIE
                    mujoco.mj_step(m, d)
                    tl.append(self.tilt(d))
                    vz.append(d.qvel[self.rv + 2])
                    fmax = max(fmax, float(np.abs(d.actuator_force[self.ctl]).max()))
                out.update(z_lying_cm=round(100 * d.xpos[self.base][2], 1), belly=self.belly(d),
                           max_tilt_deg=round(max(tl), 1), max_descent_mps=round(-min(vz), 3),
                           max_torque_nm=round(fmax, 2), pose_err_rad=round(float(np.abs(d.qpos[self.qadr] - LIE).max()), 3))
            z_before = d.xpos[self.base][2]
            self.pd(0.0, ESTOP_KD)
            d.ctrl[self.ctl] = 0.0
            zs, vz = [], []
            for _ in range(int(2.0 / self.dt)):
                mujoco.mj_step(m, d)
                zs.append(d.xpos[self.base][2])
                vz.append(d.qvel[self.rv + 2])
            out.update(drop_after_off_cm=round(100 * (z_before - min(zs)), 2), impact_mps=round(-min(vz), 3),
                       z_off_cm=round(100 * zs[-1], 1), tilt_off_deg=round(self.tilt(d), 1))
            q1 = d.qpos[self.qadr].copy()
            self.pd(INIT_KP * kps, INIT_KD)
            for k in range(int(2.0 / self.dt)):
                a = k * self.dt / 2.0
                d.ctrl[self.ctl] = (1 - a) * q1 + a * STANCE
                mujoco.mj_step(m, d)
            self.pd(STAND_KP * kps, STAND_KD)
            d.ctrl[self.ctl] = STANCE
            step(1.0)
            out.update(restand_z_cm=round(100 * d.xpos[self.base][2], 1), restand_tilt_deg=round(self.tilt(d), 1))
            return out
        finally:
            m.geom_friction[:, 0] = self.fric0
            m.body_mass[self.base] = self.mass0


def cases():
    rng = np.random.default_rng(3)
    c = [("nominal", {}), ("blend 1.5 s", dict(T=1.5)), ("blend 3.5 s", dict(T=3.5)), ("friction x0.5", dict(fric=0.5)),
         ("friction x1.5", dict(fric=1.5)), ("mass +0.3 kg", dict(mass=0.3)), ("kp x0.85", dict(kps=0.85)),
         ("pitch +3 deg", dict(pitch0=0.05)), ("pitch -3 deg", dict(pitch0=-0.05))]
    c += [("stance offset %d" % i, dict(off=rng.uniform(-0.1, 0.1, 12))) for i in range(4)]
    c += [("harsh", dict(fric=0.6, mass=0.3, kps=0.85, off=rng.uniform(-0.1, 0.1, 12)))]
    return c


def passed(r):
    return (r["belly"] and r["max_tilt_deg"] < 10 and r["drop_after_off_cm"] < 2.0 and r["restand_tilt_deg"] < 10)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", help="write the full result here")
    a = ap.parse_args()
    os.environ.setdefault("FORCE_CPU", "1")
    sim = Sim()
    res = {"lie_pose": so.LIE_POSE, "init_s": so.LIEDOWN_INIT_S, "r3_baseline": sim.run(r3=True), "cases": {}}
    for name, kw in cases():
        r = sim.run(**kw)
        r["pass"] = passed(r)
        res["cases"][name] = r
        print("%-17s %s" % (name, json.dumps(r)), flush=True)
    print("R3 baseline       " + json.dumps(res["r3_baseline"]))
    res["all_pass"] = all(r["pass"] for r in res["cases"].values())
    worst = {k: max(r[k] for r in res["cases"].values())
             for k in ("max_tilt_deg", "max_descent_mps", "drop_after_off_cm", "restand_tilt_deg", "max_torque_nm")}
    res["worst"] = worst
    print("worst " + json.dumps(worst) + "  ALL PASS" if res["all_pass"] else "  FAIL")
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1))
    return 0 if res["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
