#!/usr/bin/env python3
"""Fast-gait screen: exported policy JSON on the production C++ inference/reference
headers (host.cpp, a copy of foundation/cpp/export_locomotion_controller.cpp with a
configurable speed cap / heading hold), MuJoCo 520 Hz nominal full-hull model from
foundation/scripts/check_spider_handoff.make_model. Adapted from
foundation/scripts/check_export_locomotion.py. No robot access.

Scenario knobs: command, speed cap, heading hold, ball, gyro noise/bias/x-pulses,
foot friction scale, kp scale, added base mass.
"""
from __future__ import annotations
import argparse, importlib.util, json, os, subprocess, sys
from collections import deque
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path('/Users/shrish/workspace/cs123')
HERE = Path(os.environ.get('FG_SIM_DIR', str(Path(__file__).resolve().parent)))  # dir holding the fast-gait 'host' binary (fc/notes/fast-gait.md)
sys.path.insert(0, str(Path(__file__).resolve().parent)); import fcimu
NAMES = [f'leg_{leg}_{j}' for leg in ('front_r', 'front_l', 'back_r', 'back_l') for j in (1, 2, 3)]
spec = importlib.util.spec_from_file_location('hand', ROOT / 'foundation/scripts/check_spider_handoff.py')
hand = importlib.util.module_from_spec(spec); sys.modules['hand'] = hand; spec.loader.exec_module(hand)
LIM = json.loads((ROOT / 'foundation/configs/class-driver-joint-limits.json').read_text())['joint_limits']
LOWER = np.array([LIM[n][0] for n in NAMES]); UPPER = np.array([LIM[n][1] for n in NAMES])


def quat_from_rpy(r, p, y):
    cr, cp, cy = np.cos(np.array([r, p, y]) / 2); sr, sp, sy = np.sin(np.array([r, p, y]) / 2)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def rpy(q):
    w, x, y, z = q
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)); pitch = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
    return roll, pitch


QOFF = np.array([np.cos(np.radians(-135)/2), 0, np.sin(np.radians(-135)/2), 0])  # wxyz, sensor = body * QOFF
QOFF_INV = QOFF * np.array([1, -1, -1, -1])
c_, s_ = np.cos(np.radians(-135)), np.sin(np.radians(-135)); ROFF = np.array([[c_, 0, s_], [0, 1, 0], [-s_, 0, c_]])
def qmul(a, b):
    r = np.zeros(4); mujoco.mju_mulQuat(r, np.asarray(a, float), np.asarray(b, float)); return r
def xyzw2wxyz(q): return np.array([q[3], q[0], q[1], q[2]])


class ImuChain:
    """Sensor-frame BNO model: true rates/specific force + X faults, then either the stock
    fused output (weak-accel Mahony kp 0.2 fitted to ks4/ks6) or FC_IMU_FILTER (driver header)."""
    def __init__(self, sc, rng, mode):
        self.sc, self.rng, self.mode = sc, rng, mode
        self.bno = fcimu.Filter(kp=0.2, ki=0.0, quiet_rate=0.0, spike_thresh=1e9, freeze_s=1e9)
        self.fc = fcimu.Filter(**sc.get('fc_kw', {}))
        self.pulse_amp = 0; self.pulse_left = 0; self.tau = 0.01
        self.next_freeze = sc.get('freeze_every', 0) or 1e9; self.freeze_left = 0
        self.gf = None; self.af = None; self.qbno_f = None
        self.out_q = np.array([1, 0, 0, 0.]); self.out_w = np.zeros(3); self.err = []
    def step(self, t, dt, q_body, w_body, f_world, fault_on):
        sc, rng = self.sc, self.rng
        qs = qmul(q_body, QOFF)
        R = np.zeros(9); mujoco.mju_quat2Mat(R, qs); R = R.reshape(3, 3)
        acc = R.T @ f_world + rng.normal(0, 0.03, 3)
        self.an = getattr(self, 'an', []); self.an.append(float(np.linalg.norm(f_world)))
        g = ROFF.T @ w_body + rng.normal(0, 0.002, 3)
        freezing = False
        if fault_on:
            g[0] += sc.get('bias_x', 0.0)
            if self.pulse_left <= 0 and rng.random() < sc.get('pulse_hz', 0) * dt:
                self.pulse_amp = rng.uniform(.03, .3); self.pulse_left = rng.uniform(.015, .04); self.tau = self.pulse_left / 2
            if self.pulse_left > 0:
                g[0] -= self.pulse_amp; self.pulse_amp *= np.exp(-dt / self.tau); self.pulse_left -= dt
            if t >= self.next_freeze and self.freeze_left <= 0:
                self.freeze_left = rng.uniform(.3, 1.5); self.next_freeze = t + sc['freeze_every']
                self.gf = g.copy(); self.af = acc.copy(); self.qbno_f = None
            if self.freeze_left > 0:
                freezing = True; self.freeze_left -= dt; g = self.gf.copy()
                if sc.get('freeze_accel'): acc = self.af.copy()
        one = np.ones(1, np.int32)
        qb, _, _ = self.bno.run(np.array([dt]), g[None], acc[None], one)
        qf, wf, _ = self.fc.run(np.array([dt]), g[None], acc[None], one)
        if self.mode == 'truth':
            q_s = qs; w_s = ROFF.T @ w_body
        elif self.mode == 'filter':
            q_s = xyzw2wxyz(qf[0]); w_s = wf[0]
        else:
            q_s = xyzw2wxyz(qb[0]); w_s = g
            if freezing:   # chip output frozen: RV repeats too
                if self.qbno_f is None: self.qbno_f = self.out_qs if hasattr(self, 'out_qs') else q_s
                q_s = self.qbno_f
            self.out_qs = q_s
        self.out_q = qmul(q_s, QOFF_INV); self.out_w = ROFF @ w_s
        up_t = R.T @ np.array([0, 0, 1.]); Ro = np.zeros(9); mujoco.mju_quat2Mat(Ro, qmul(self.out_q, QOFF)); up_o = Ro.reshape(3, 3).T @ np.array([0, 0, 1.])
        self.err.append(float(np.degrees(np.arccos(np.clip(up_t @ up_o, -1, 1)))))
        x, y, z, w = qf[0]; uf = np.array([2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z])
        self.errf = getattr(self, 'errf', []); self.errf.append(float(np.degrees(np.arccos(np.clip(up_t @ uf, -1, 1)))))
        self.tilt_t = getattr(self, 'tilt_t', []); self.tilt_t.append(up_t.copy())


def run(policy, sc, replicas=3, seed=7, seconds=6.0, warmup=2.0):
    meta = json.loads(Path(policy).read_text())
    ball = sc.get('ball', False)
    mass_add = sc.get('mass_add', 0.0)

    def scene(s):
        if ball:
            br, bm = sc.get('ball_r', 0.11), sc.get('ball_m', 0.43)
            b = s.worldbody.add_body(name='ball', pos=[0.31 + br, 0, br])
            b.add_freejoint(name='ball_free')
            b.add_geom(name='ball_geom', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[br, 0, 0], mass=bm,
                       contype=1, conaffinity=1, friction=[0.6, 0.005, 0.0001], condim=6)
    model, _ = hand.make_model('full_hulls', configure_scene=scene)
    base = model.body('base_link').id
    if mass_add: model.body_mass[base] += mass_add
    fs = sc.get('friction_scale', 1.0)
    if fs != 1.0:
        model.geom_friction[:, 0] *= fs
    qadr = model.jnt_qposadr[[model.joint(n).id for n in NAMES]]
    controls = np.array([int(np.flatnonzero(model.actuator_trnid[:, 0] == model.joint(n).id)[0]) for n in NAMES])
    rj = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and model.jnt_bodyid[j] == base][0]
    rq, rv = int(model.jnt_qposadr[rj]), int(model.jnt_dofadr[rj])
    if ball:
        bj = model.joint('ball_free').id; bq = int(model.jnt_qposadr[bj])
    sites = [model.site(f'leg_{l}_3_foot_site').id for l in ('front_r', 'front_l', 'back_r', 'back_l')]
    foot_bodies = [int(model.site_bodyid[s]) for s in sites]
    terrain = model.geom('terrain').id
    rng = np.random.default_rng(seed)
    data, delayed, inits = [], [], []
    for i in range(replicas):
        r, p = (0, 0) if i == 0 else rng.uniform(-.02, .02, 2)
        off = np.zeros(12) if i == 0 else rng.uniform(-.01, .01, 12)
        d = mujoco.MjData(model)
        d.qpos[rq:rq+3] = [0, 0, .18]; d.qpos[rq+3:rq+7] = quat_from_rpy(r, p, 0)
        jp = np.asarray(meta['default_joint_pos']) + off; d.qpos[qadr] = jp
        mujoco.mj_forward(model, d); data.append(d); inits.append(jp)
        t0 = np.clip(jp, LOWER, UPPER); delayed.append(deque([(-1., t0.copy()), (0., t0.copy())]))
    env = dict(os.environ, FG_SPEED_CAP=str(sc.get('cap', 1.5)), FG_HH='1' if sc.get('hh', True) else '0')
    proc = subprocess.Popen([str(HERE / 'host'), str(policy), str(replicas), '--no-jump'], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1, env=env)
    kp_scale = sc.get('kp_scale', 1.0)
    gyro_sigma = sc.get('gyro_sigma', 0.0); gyro_bias = np.asarray(sc.get('gyro_bias', [0, 0, 0]), float)
    pulse_rate = sc.get('gyro_x_pulse_hz', 0.0); pulse_amp = sc.get('gyro_x_pulse_amp', 0.0)
    pulse_left = [0]*replicas
    sched = sc.get('schedule')  # [[t_rel_s, [vx,vy,wz]], ...]
    if sched: seconds = sc.get('seconds', seconds)
    tilt_off = sc.get('quat_tilt_deg')
    if tilt_off:
        qo = np.array(quat_from_rpy(np.radians(tilt_off[0]), np.radians(tilt_off[1]), 0))
    imus = [ImuChain(sc, np.random.default_rng(seed + 100 + i), sc.get('imu_mode', 'stock')) for i in range(replicas)]
    vprev = [d.qvel[rv:rv+3].copy() for d in data]
    fault_t = sc.get('fault_start', 2.0)
    act_ticks = round(4.2*520); seg_ticks = round(seconds*520); total = act_ticks + seg_ticks
    active = [True]*replicas; outputs = [None]*replicas; rows = []; falls = []
    ball_start = [None]*replicas
    cmd = sc['cmd']
    for tick in range(total):
        t = tick/520; seg = tick - act_ticks
        requested = cmd if seg >= 0 else [0, 0, 0]
        if sched and seg >= 0:
            requested = [c for t_, c in sched if t_ <= seg/520][-1]
        if tick % 5 == 0:
            for i, d in enumerate(data):
                if not active[i]: continue
                v = d.qvel[rv:rv+3].copy(); fw = (v - vprev[i]) / (5/520) + np.array([0, 0, 9.81]); vprev[i] = v
                imus[i].step(t, 5/520, d.qpos[rq+3:rq+7].copy(), d.qvel[rv+3:rv+6].copy(), fw, t >= fault_t)
        if tick >= 1040 and (tick-1040) % 10 == 0:
            states = []
            for i, d in enumerate(data):
                q = d.qpos[rq+3:rq+7]
                tilt = float(np.arccos(np.clip(1-2*(q[1]**2+q[2]**2), -1, 1)))
                if active[i] and (not np.isfinite(d.qpos).all() or tilt > 1.0 or d.qpos[rq+2] < 0.05):
                    falls.append(dict(replica=i, t=round(t - 4.2, 2), tilt=round(tilt, 2), z=round(float(d.qpos[rq+2]), 3)))
                    active[i] = False
                if active[i]:
                    g = d.qvel[rv+3:rv+6].copy()
                    g = g + gyro_bias + rng.normal(0, gyro_sigma, 3) if gyro_sigma or gyro_bias.any() else g
                    if pulse_rate:
                        if pulse_left[i] == 0 and rng.random() < pulse_rate/52:
                            pulse_left[i] = 2  # ~40 ms pulse
                        if pulse_left[i] > 0:
                            g[0] += pulse_amp * (1 if rng.random() < .5 else -1); pulse_left[i] -= 1
                    qs = q.copy()
                    if tilt_off:  # IMU orientation error: body-frame tilt offset
                        qs = np.zeros(4); mujoco.mju_mulQuat(qs, q, qo)
                    if sc.get('imu_mode'):
                        g = imus[i].out_w; qs = imus[i].out_q
                    states.append(dict(elapsed_s=t, command=requested, joint_pos=d.qpos[qadr].tolist(),
                                       angular_velocity=list(map(float, g)), quaternion=list(map(float, qs))))
                else:
                    states.append(None)
            if not any(active): break
            proc.stdin.write(json.dumps(dict(op='infer', states=states)) + '\n'); proc.stdin.flush()
            line = proc.stdout.readline()
            if not line: raise RuntimeError(proc.stderr.read())
            outputs = json.loads(line)
        for i, d in enumerate(data):
            if not active[i]: continue
            if tick < 1040:
                a = t/2; target = (1-a)*inits[i] + a*np.asarray(meta['default_joint_pos']); kp, kd = 7.5, .25
            else:
                target = np.asarray(outputs[i]['target']); kp, kd = meta['kp']*kp_scale, meta['kd']
            final = np.clip(target, LOWER, UPPER)
            dl = delayed[i]; dl.append((t, final.copy())); cut = t - .016
            while len(dl) > 2 and dl[1][0] <= cut: dl.popleft()
            (t0, v0), (t1, v1) = dl[0], dl[1]; fr = np.clip((cut-t0)/(t1-t0), 0, 1)
            d.ctrl[controls] = (1-fr)*v0 + fr*v1
            model.actuator_gainprm[controls, 0] = kp; model.actuator_biasprm[controls, 1] = -kp; model.actuator_biasprm[controls, 2] = -kd
            if seg == 0 and ball: ball_start[i] = d.qpos[bq:bq+3].copy()
            if seg >= 0 and seg % 10 == 0:
                q = d.qpos[rq+3:rq+7]; R = d.xmat[base].reshape(3, 3)
                lin = R.T @ d.qvel[rv:rv+3]
                fc = [False]*4
                for c in d.contact[:d.ncon]:
                    a_, b_ = int(c.geom1), int(c.geom2)
                    if terrain not in (a_, b_): continue
                    cb = int(model.geom_bodyid[b_ if a_ == terrain else a_])
                    for j, fb in enumerate(foot_bodies):
                        if cb == fb: fc[j] = True
                sp = []
                for s_ in sites:
                    v = np.zeros(6); mujoco.mj_objectVelocity(model, d, mujoco.mjtObj.mjOBJ_SITE, s_, v, 0); sp.append(float(np.linalg.norm(v[3:5])))
                roll, pitch = rpy(q)
                rows.append(dict(i=i, ts=seg/520, scored=seg/520 >= warmup, v=[lin[0], lin[1], d.qvel[rv+5]],
                                 eff=outputs[i]['effective_command'], slip=[s for s, c in zip(sp, fc) if c],
                                 roll=roll, pitch=pitch, pos=d.qpos[rq:rq+3].copy(),
                                 ball=d.qpos[bq:bq+3].copy() if ball else None))
            mujoco.mj_step(model, d)
    proc.stdin.close(); proc.wait(timeout=10)
    sel = [r for r in rows if r['scored']]
    errs = [e for im in imus for e in im.err[round(fault_t*104):]]
    ef_ = [e for im in imus for e in im.errf[round(5*104):]]
    out = dict(filt_err_p95=round(float(np.percentile(ef_, 95)), 1), filt_err_max=round(max(ef_), 1), name=sc['name'], imu_mode=sc.get('imu_mode'), imu_err_max_deg=round(max(errs), 1) if errs else None, imu_err_p95_deg=round(float(np.percentile(errs, 95)), 1) if errs else None, acc_norm_p5_p95=[round(float(np.percentile([a for im in imus for a in im.an[round(5*104):]], q)), 1) for q in (5, 50, 95)], cmd=cmd, cap=sc.get('cap', 1.5), falls=falls, n_rep=replicas)
    if sel:
        v = np.array([r['v'] for r in sel]); e = np.array([r['eff'] for r in sel])
        slips = [s for r in sel for s in r['slip']]
        out.update(v=v.mean(0).round(3).tolist(), eff=e.mean(0).round(3).tolist(),
                   xy_err=round(float(np.linalg.norm(v[:, :2]-e[:, :2], axis=1).mean()), 3),
                   yaw_err=round(float(np.abs(v[:, 2]-e[:, 2]).mean()), 3),
                   slip=round(float(np.mean(slips)), 3) if slips else None,
                   roll_rms_deg=round(float(np.degrees(np.sqrt(np.mean([r['roll']**2 for r in sel])))), 1),
                   pitch_rms_deg=round(float(np.degrees(np.sqrt(np.mean([r['pitch']**2 for r in sel])))), 1),
                   tilt_max_deg=round(float(np.degrees(max(max(abs(r['roll']), abs(r['pitch'])) for r in sel))), 1))
        # straight-line progress over scored window (per replica): world displacement / time
        out['max_tilt_all_deg'] = round(float(np.degrees(max(max(abs(r['roll']), abs(r['pitch'])) for r in rows))), 1)
        prog = []
        for i in range(replicas):
            rr = [r for r in sel if r['i'] == i]
            if len(rr) > 2: prog.append(np.linalg.norm(rr[-1]['pos'][:2]-rr[0]['pos'][:2])/(rr[-1]['ts']-rr[0]['ts']))
        out['world_speed'] = round(float(np.mean(prog)), 3) if prog else None
        if sc.get('stop_at') is not None:
            T = sc['stop_at']; dist = []; tset = []
            for i in range(replicas):
                rr = [r for r in rows if r['i'] == i and r['ts'] >= T]
                if not rr: continue
                sp = [np.hypot(r['v'][0], r['v'][1]) for r in rr]
                k = next((j for j in range(len(sp)) if all(x < 0.05 for x in sp[j:j+13])), len(sp)-1)
                dist.append(float(np.linalg.norm(rr[k]['pos'][:2]-rr[0]['pos'][:2]))); tset.append(rr[k]['ts']-T)
            out['stop_dist_m'] = round(max(dist), 3) if dist else None
            out['stop_time_s'] = round(max(tset), 2) if tset else None
        if ball:
            bd = []
            for i in range(replicas):
                rr = [r for r in rows if r['i'] == i]
                if rr: bd.append(float(np.linalg.norm(rr[-1]['ball'][:2]-rr[0]['ball'][:2])))
            out['ball_moved_m'] = round(float(np.mean(bd)), 2)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('policy'); ap.add_argument('scenarios', help='JSON list'); ap.add_argument('out')
    ap.add_argument('--replicas', type=int, default=3); ap.add_argument('--seconds', type=float, default=6.0)
    a = ap.parse_args()
    res = []
    for sc in json.loads(Path(a.scenarios).read_text()):
        r = run(a.policy, sc, replicas=a.replicas, seconds=a.seconds); res.append(r); print(json.dumps(r), flush=True)
    Path(a.out).write_text(json.dumps(res, indent=1))
