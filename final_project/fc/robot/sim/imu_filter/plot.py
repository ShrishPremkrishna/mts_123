import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, sys
fig, ax = plt.subplots(2, 1, figsize=(12, 6.5), sharey=True)
for a, k in zip(ax, ['ks4', 'ks6']):
    d = np.load(f'data/{k}_rep100.npz'); t = d['t']; mv = d['moving']
    a.fill_between(t, 0, 40, where=mv, color='0.85', step='mid', label='joints moving (FK truth unreliable)')
    a.plot(t, d['eb'], color='#c0392b', lw=0.6, label='BNO fused (stock), error vs foot-plane truth')
    a.plot(t, d['ef'], color='#1f6fb2', lw=0.8, label='FC_IMU_FILTER (driver header), same input')
    a.set_title(f'{k}: tilt error (deg), recorded gyro replayed at 100 Hz'); a.set_ylim(0, 40); a.grid(alpha=.3)
ax[0].legend(loc='upper left', fontsize=8); ax[1].set_xlabel('session time (s)')
plt.tight_layout(); plt.savefig(sys.argv[1], dpi=110)
