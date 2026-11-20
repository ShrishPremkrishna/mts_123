"""Deploy-artifact closed loop on a synthetic test stand (no mjlab).

Controller semantics from neural_controller.cpp: 48-dim frames newest-first, 20 frames;
last_action = raw (pre-clamp) output; target = clamp(0.75 a + default, limits); zero command
=> gait-reference dims 0; base pinned (gyro 0, gravity (0,0,-1)). Plant: each joint an
independent PD-driven inertia I (kp 5.5, kd 0.25, |tau|<=3 Nm, frictionloss 0.125 Nm),
1 kHz integration, pure transport delay L on the target. Sensor noise: joint 0.002 rad,
gyro 0.01 rad/s (hardware-like, not the training noise). Kick: 0.15 rad on one leg at t=1 s.
"""
import json, sys, numpy as np
sys.path.insert(0, "/home/ubuntu/mts_123/pupper-mjlab/src")
from mjlab.tasks.pupper.export import json_forward

def run(pol, I, L_ms, rate_hz=50.0, secs=8.0, seed=0, kick=0.15, noise=(0.002, 0.01), train_noise=False):
    rng = np.random.default_rng(seed)
    d = np.array(pol["default_joint_pos"]); lo = np.array(pol["joint_lower_limits"]); hi = np.array(pol["joint_upper_limits"])
    H, F = pol["observation_history"], pol["single_observation_size"]
    obs = np.zeros(H * F); obs[3 + 2::F] = -1.0  # gravity z in every frame (on_activate)
    q = d.copy(); qd = np.zeros(12); a = np.zeros(12)
    dt = 1e-3; nctrl = int(round(1000 / rate_hz)); L = int(round(L_ms))
    tq = [d.copy()] * (L + 1)
    A, Q = [], []
    for k in range(int(secs / dt)):
        if k % nctrl == 0:
            fr = obs[:F]
            jn, gn = (0.05, 0.1) if train_noise else noise
            fr[0:3] = rng.uniform(-gn, gn, 3)
            g = np.array([0, 0, -1.0]) + (rng.uniform(-0.05, 0.05, 3) if train_noise else 0); fr[3:6] = g / np.linalg.norm(g)
            fr[6:9] = 0; fr[9:12] = [0, 0, 1]
            fr[12:24] = q - d + rng.uniform(-jn, jn, 12)
            fr[36:48] = 0
            a = json_forward(pol, obs[None])[0]
            obs = np.roll(obs, F)  # newest-first: shift right, frame 0 = next
            obs[24:36] = a + (rng.uniform(-0.01, 0.01, 12) if train_noise else 0)
            A.append(a.copy()); Q.append(q.copy())
            target = np.clip(0.75 * a + d, lo, hi)
        if k == int(1.0 / dt): q[9:12] += kick  # perturb back-left leg
        tq.append(target if k >= 0 and 'target' in dir() else d); tq = tq[-(L + 1):]
        u = tq[0]
        tau = np.clip(5.5 * (u - q) - 0.25 * qd, -3, 3)
        tau = tau - np.clip(0.125 * np.sign(qd) * (np.abs(qd) > 1e-3), -abs(tau) - 1, abs(tau) + 1) * 0  # friction off-sign handled below
        acc = (tau - 0.125 * np.tanh(qd / 0.01)) / I
        qd += acc * dt; q += qd * dt
    A, Q = np.array(A), np.array(Q)
    s = int(1.5 * rate_hz)  # after the kick
    late = slice(int(5 * rate_hz), None)
    return dict(act_rate_after_kick=float(np.abs(np.diff(A[s:], axis=0)).mean()),
                act_rate_late=float(np.abs(np.diff(A[late], axis=0)).mean()),
                q_std_late=float(Q[late].std(0).mean()))

pols = {n: json.load(open(p)) for n, p in [("p3", "/home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/policy.json"),
                                           ("fasx", "/tmp/fasx/policy.json"  # W&B photon_mz-stanford-university/mjlab/fasxqy4z policy.json)]}
print(f"{'policy':5s} {'I':>6s} {'L_ms':>4s} {'rate':>4s} {'noise':>5s} {'rate_kick':>9s} {'rate_late':>9s} {'q_std_late':>10s}")
for name, pol in pols.items():
    for I in (0.002, 0.006):
        for L in (16, 36, 56):
            for tn in (False, True):
                r = run(pol, I, L, train_noise=tn)
                print(f"{name:5s} {I:6.3f} {L:4d} {50:4d} {'train' if tn else 'hw':>5s} {r['act_rate_after_kick']:9.4f} {r['act_rate_late']:9.4f} {r['q_std_late']:10.4f}", flush=True)
    r = run(pol, 0.006, 16, rate_hz=40.0); print(f"{name:5s} 0.006   16   40    hw {r['act_rate_after_kick']:9.4f} {r['act_rate_late']:9.4f} {r['q_std_late']:10.4f}")
