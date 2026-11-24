"""Usage: replay.py <dir with fcimu.py> <name> <session.npz> <fk.npz> <t0,t1 clean still window> [rate_hz] [params json] [out.npz]
Replay recorded faulty-gyro sessions through the driver filter (exact C++ header via ctypes).
Truth = foot-plane gravity from joint FK (offset-calibrated to the BNO in a clean still window).
Accel is synthetic (the stock driver never enabled it): truth gravity + noise (+ gait-like
oscillation while joints move)."""
import sys, json, numpy as np
sys.path.insert(0, sys.argv[1]); import fcimu
name, npz, fk = sys.argv[2], sys.argv[3], sys.argv[4]
calib = [float(x) for x in sys.argv[5].split(',')]    # clean still window for the FK->BNO offset
rate_hz = float(sys.argv[6]) if len(sys.argv) > 6 else 0  # 0 = native ~240 Hz, else resample
kw = json.loads(sys.argv[7]) if len(sys.argv) > 7 else {}
D = np.load(npz, allow_pickle=True); F = np.load(fk)
imu = D['imu']; t0 = imu[0, 0]; t = imu[:, 0] - t0; st = imu[:, 8]
if rate_hz:
    idx = np.flatnonzero(np.diff(np.floor(t * rate_hz), prepend=-1) > 0); imu = imu[idx]; t = t[idx]; st = st[idx]
q = imu[:, 1:5]; w = imu[:, 5:8]
c, s = np.cos(np.radians(-135)), np.sin(np.radians(-135)); Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
gs = w @ Ry                                  # sensor-frame gyro (R_off^T w)
def up_body(qx):
    x, y, z, ww = qx.T; return np.stack([2*(x*z-ww*y), 2*(y*z+ww*x), ww*ww-x*x-y*y+z*z], 1)
ub = up_body(q); ub /= np.linalg.norm(ub, axis=1)[:, None]
ft = F['t'] - t0; fu = np.stack([np.interp(t, ft, F['up'][:, i]) for i in range(3)], 1); fu /= np.linalg.norm(fu, axis=1)[:, None]
jv = np.interp(t, ft, F['jv'])
m = (t >= calib[0]) & (t < calib[1])
a_, b_ = fu[m].mean(0), ub[m].mean(0); a_ /= np.linalg.norm(a_); b_ /= np.linalg.norm(b_)
ax = np.cross(a_, b_); sn = np.linalg.norm(ax); ang = np.arctan2(sn, a_ @ b_); ax = ax / sn if sn > 0 else ax
K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]); Rc = np.eye(3) + np.sin(ang)*K + (1-np.cos(ang))*K@K
truth = fu @ Rc.T
# smooth truth during motion is unknowable; keep as is
rng = np.random.default_rng(1)
moving = jv > 0.2
acc_s = 9.80665 * truth @ Ry               # sensor frame: R_off^T up
acc_s += rng.normal(0, 0.03, acc_s.shape)
osc = np.stack([2.0*np.sin(2*np.pi*2.5*t), np.zeros_like(t), 2.0*np.sin(2*np.pi*2.5*t + 1.0)], 1) @ Ry
acc_s[moving] += osc[moving] + rng.normal(0, 1.0, (moving.sum(), 3))
dt = np.diff(st, prepend=st[0]); dt[0] = 0
f = fcimu.Filter(**kw)
qf, rf, fl = f.run(dt, gs, acc_s, np.ones(len(t), np.int32))
x, y, z, ww = qf.T
uf_s = np.stack([2*(x*z-ww*y), 2*(y*z+ww*x), ww*ww-x*x-y*y+z*z], 1)   # up in sensor frame
uf = uf_s @ Ry.T                                                        # body frame
ang_err = lambda u: np.degrees(np.arccos(np.clip((u * truth).sum(1) / np.linalg.norm(u, axis=1), -1, 1)))
eb, ef = ang_err(ub), ang_err(uf)
still = ~moving & (t > 5)
tm = t[moving]
near = np.zeros(len(t), bool)
if len(tm):
    j = np.searchsorted(tm, t); lo = tm[np.clip(j-1, 0, len(tm)-1)]; hi = tm[np.clip(j, 0, len(tm)-1)]
    near = (np.abs(t-lo) < 3) | (np.abs(hi-t) < 3)
settled = still & ~near
res = dict(session=name, rate=rate_hz or 'native', n=len(t), params=dict(fcimu.DEFAULTS, **kw),
           calib_offset_deg=round(float(np.degrees(ang)), 2),
           bno_err_still_max=round(float(eb[still].max()), 1), bno_err_still_p99=round(float(np.percentile(eb[still], 99)), 1),
           filt_err_still_max=round(float(ef[still].max()), 2), filt_err_still_p99=round(float(np.percentile(ef[still], 99)), 2),
           filt_err_still_mean=round(float(ef[still].mean()), 2),
           bno_err_moving_max=round(float(eb[moving].max()), 1) if moving.any() else None,
           filt_err_moving_max=round(float(ef[moving].max()), 2) if moving.any() else None,
           filt_err_moving_p99=round(float(np.percentile(ef[moving], 99)), 2) if moving.any() else None,
           bno_err_settled_max=round(float(eb[settled].max()), 1), filt_err_settled_max=round(float(ef[settled].max()), 2),
           filt_err_settled_p99=round(float(np.percentile(ef[settled], 99)), 2), frac_settled=round(float(settled.mean()), 2),
           frac_still=round(float(still.mean()), 2),
           quiet_rawx_std=round(float(gs[(fl & 4) > 0, 0].std()), 4), quiet_outx_std=round(float(rf[(fl & 4) > 0, 0].std()), 4),
           stats=f.stats())
print(json.dumps(res))
if len(sys.argv) > 8:
    np.savez(sys.argv[8], t=t, eb=eb, ef=ef, moving=moving, rawx=gs[:, 0], outx=rf[:, 0], fl=fl)
