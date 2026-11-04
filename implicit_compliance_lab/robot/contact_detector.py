"""NumPy-only implicit contact detector for Pupper's front-right leg.

No ROS and no PyTorch: the lab reports that PyTorch inference on the Raspberry Pi 5
overloaded it, so the exported JointStateCNN runs here with NumPy (about 4 M
multiply-adds per call at W = 200).

Pieces, each usable alone:
    JointStateCNN   - forward pass of the exported .npz model
    JointStateBuffer- ring buffer of the latest joint_states samples
    Hysteresis      - debounce: N consecutive frames above p_on to set, below p_off to clear
    PauseOnContact  - "pause the trajectory for 1 s on contact" state machine
    PhaseBaseline   - effort residual against a free-swing recording (robot-calibrated detector)

Calibrate the residual detector from a free-swing log:
    python3 contact_detector.py calibrate free_swing.csv free_swing_baseline.npz
"""

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

JOINTS = ('leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3')
BN_EPS = 1e-5


class JointStateCNN:
    """Eval-mode forward pass of the notebook's JointStateCNN from an exported .npz."""

    def __init__(self, path):
        z = np.load(path, allow_pickle=False)
        self.window = int(z['window_size'])
        self.channels = z['channels'].astype(int)
        self.mean = z['mean'].astype(np.float32)
        self.std = z['std'].astype(np.float32)
        self.incline_classes = z['incline_classes'].tolist()
        self.material_classes = z['material_classes'].tolist()
        self.conv = []
        for i in (1, 2, 3):
            # Fold batch norm (running statistics) into the convolution.
            scale = z[f'bn{i}.weight'] / np.sqrt(z[f'bn{i}.running_var'] + BN_EPS)
            w = z[f'conv{i}.weight'] * scale[:, None, None]          # [out, in, k]
            b = (z[f'conv{i}.bias'] - z[f'bn{i}.running_mean']) * scale + z[f'bn{i}.bias']
            self.conv.append((w.reshape(w.shape[0], -1).astype(np.float32), b.astype(np.float32), w.shape[2]))
        self.fc1 = (z['fc1.weight'].astype(np.float32), z['fc1.bias'].astype(np.float32))
        self.heads = {k: (z[f'{k}_head.weight'].astype(np.float32), z[f'{k}_head.bias'].astype(np.float32))
                      for k in ('contact', 'incline', 'material')}

    def __call__(self, x):
        """x: [W, 9] or [B, W, 9] raw joint states in dataset column order.

        Returns contact probability [B], incline logits [B, 6], material logits [B, 6].
        """
        x = np.asarray(x, np.float32)
        if x.ndim == 2:
            x = x[None]
        x = (x[:, -self.window:, self.channels] - self.mean) / self.std
        h = x.transpose(0, 2, 1)                                      # [B, C, L]
        for w, b, k in self.conv:
            p = k // 2
            hp = np.pad(h, ((0, 0), (0, 0), (p, p)))
            cols = sliding_window_view(hp, k, axis=2)                 # [B, C, L, k]
            cols = cols.transpose(0, 2, 1, 3).reshape(h.shape[0], h.shape[2], -1)  # [B, L, C*k]
            h = np.maximum(cols @ w.T + b, 0).transpose(0, 2, 1)     # conv + BN + ReLU -> [B, out, L]
            n = h.shape[2] // 2
            h = h[:, :, :2 * n].reshape(h.shape[0], h.shape[1], n, 2).max(axis=3)  # MaxPool1d(2)
        h = np.maximum(h.reshape(h.shape[0], -1) @ self.fc1[0].T + self.fc1[1], 0)
        logit = h @ self.heads['contact'][0].T + self.heads['contact'][1]
        return (1.0 / (1.0 + np.exp(-np.clip(logit[:, 0], -60.0, 60.0))),
                h @ self.heads['incline'][0].T + self.heads['incline'][1],
                h @ self.heads['material'][0].T + self.heads['material'][1])


class JointStateBuffer:
    """Fixed-size ring buffer of [position(3), velocity(3), effort(3)] rows."""

    def __init__(self, size):
        self.size = size
        self.data = np.zeros((size, 9), np.float32)
        self.count = 0

    def push(self, position, velocity, effort):
        self.data[self.count % self.size] = (*position, *velocity, *effort)
        self.count += 1

    def full(self):
        return self.count >= self.size

    def window(self):
        """The latest `size` rows, oldest first."""
        i = self.count % self.size
        return np.concatenate([self.data[i:], self.data[:i]])


class Hysteresis:
    """Contact is set after n_on frames with p >= p_on and cleared after n_off frames with p < p_off."""

    def __init__(self, p_on=0.5, p_off=0.3, n_on=3, n_off=5):
        self.p_on, self.p_off, self.n_on, self.n_off = p_on, p_off, n_on, n_off
        self.state = False
        self._run = 0

    def update(self, p):
        """Returns (state, rising_edge)."""
        if self.state:
            self._run = self._run + 1 if p < self.p_off else 0
            if self._run >= self.n_off:
                self.state, self._run = False, 0
            return self.state, False
        self._run = self._run + 1 if p >= self.p_on else 0
        if self._run >= self.n_on:
            self.state, self._run = True, 0
            return True, True
        return False, False


class PauseOnContact:
    """Pause once per touchdown.

    On a contact rising edge while armed, hold the trajectory for `pause_s`.
    After the pause the trajectory continues (the foot slides on the surface as in data
    collection). The next pause is armed only after the detector reports release
    continuously for `rearm_s` after the pause has ended. Thus a foot that stays down,
    or a release reported while the leg is held still, cannot pause the same touchdown twice.
    """

    def __init__(self, pause_s=1.0, rearm_s=0.2):
        self.pause_s = pause_s
        self.rearm_s = rearm_s
        self.armed = True
        self.pause_until = -np.inf
        self.release_since = None
        self.n_pauses = 0

    def update(self, now, contact, rising_edge):
        """Returns True while the trajectory must hold still. `now` in seconds (monotonic)."""
        paused = now < self.pause_until
        if paused or contact:
            self.release_since = None
        elif self.release_since is None:
            self.release_since = now
        if self.release_since is not None and now - self.release_since >= self.rearm_s:
            self.armed = True
        if rising_edge and self.armed and not paused:
            self.pause_until = now + self.pause_s
            self.armed = False
            self.n_pauses += 1
            paused = True
        return paused


class PhaseBaseline:
    """Effort residual against a free-swing recording of the same trajectory.

    The Lab 3 trajectory is periodic (3 time units per triangle), so the effort that a leg
    in free air needs at each point of the triangle repeats. A contact adds load, and the
    PD deflection (effort) leaves that profile. The score is the largest absolute z-score of
    the mean residual over the last `win` samples, for joints 1 and 3 (joint 2 is abduction,
    which a sagittal-plane contact barely loads).
    """

    def __init__(self, path, win=20):
        z = np.load(path, allow_pickle=False)
        self.mu, self.sd, self.win = z['mu'], z['sd'], win
        self.nb = len(self.mu)

    def bins(self, phase):
        return np.minimum((np.mod(phase, 3.0) / 3.0 * self.nb).astype(int), self.nb - 1)

    def score(self, effort, phase):
        """effort [N, 3] and trajectory time [N] (newest last). Returns a z-score >= 0."""
        k = self.bins(np.asarray(phase)[-self.win:])
        r = (np.asarray(effort)[-self.win:] - self.mu[k]) / self.sd[k]
        return float(np.abs(r.mean(axis=0))[[0, 2]].max())


def build_baseline(log_csv, out_npz, n_bins=60, skip_t=0.5, min_sd=0.02):
    """Per-phase mean and SD of the 3 efforts from a `contact_ik.py --log` free-swing CSV."""
    cols = None
    rows = []
    with open(log_csv) as f:
        header = f.readline().strip().split(',')
        cols = [header.index(f'{j}_effort') for j in JOINTS] + [header.index('traj_t')]
        for line in f:
            v = line.split(',')
            rows.append([float(v[c]) for c in cols])
    a = np.array(rows)
    a = a[a[:, 3] > skip_t]                       # skip the move to the start pose
    k = np.minimum((np.mod(a[:, 3], 3.0) / 3.0 * n_bins).astype(int), n_bins - 1)
    mu = np.array([a[k == b, :3].mean(0) for b in range(n_bins)])
    sd = np.maximum(np.array([a[k == b, :3].std(0) for b in range(n_bins)]), min_sd)
    np.savez(out_npz, mu=mu, sd=sd, source=np.array(log_csv))
    return mu, sd


if __name__ == '__main__':
    import sys
    if len(sys.argv) != 4 or sys.argv[1] != 'calibrate':
        sys.exit('usage: python3 contact_detector.py calibrate FREE_SWING_LOG.csv OUT_BASELINE.npz')
    mu, sd = build_baseline(sys.argv[2], sys.argv[3])
    print(f'wrote {sys.argv[3]}: {len(mu)} phase bins, median SD {np.median(sd, axis=0).round(3)}')
