"""Desk checks for the robot contact detector (no ROS, no robot).

    python3 test_contact_detector.py                      # state machine + speed (runs on the Pi)
    python3 test_contact_detector.py --data ~/tactile_data --part val
        # + replay recorded joint states through buffer -> model -> hysteresis at 1 kHz / 50 Hz
    python3 test_contact_detector.py --parity             # + NumPy vs PyTorch outputs (needs torch)

--part val replays only the last 20 % of each recording, the part that a model trained by
`analysis/ablation.py --split blocked` never saw.
"""

import argparse
import glob
import os
import time

import numpy as np

from contact_detector import Hysteresis, JointStateBuffer, JointStateCNN, PauseOnContact

HERE = os.path.dirname(os.path.abspath(__file__))


def test_state_machine():
    h = Hysteresis(p_on=0.5, p_off=0.3, n_on=3, n_off=5)
    seq = [0.9, 0.9, 0.2, 0.9, 0.9, 0.9, 0.4, 0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.9]
    out = [h.update(p) for p in seq]
    assert [s for s, _ in out] == [False] * 5 + [True] * 7 + [False] * 2, out
    assert [e for _, e in out].count(True) == 1 and out[5][1]

    pause = PauseOnContact(pause_s=1.0, rearm_s=0.2)
    assert pause.update(0.0, True, True)            # touchdown -> pause
    assert pause.update(0.5, False, False)          # release reported during the hold is ignored
    assert not pause.update(1.01, False, False)     # pause over, trajectory slides on
    assert not pause.update(1.05, True, True)       # same touchdown again after 40 ms release: no pause
    assert not pause.update(2.0, False, False)      # release starts
    assert not pause.update(2.1, False, False) and not pause.armed   # 0.1 s < rearm_s
    assert not pause.update(2.25, False, False) and pause.armed      # 0.25 s >= rearm_s re-arms
    assert pause.update(3.0, True, True) and pause.n_pauses == 2
    print('state machine: ok')


def bench(model, n=200):
    x = np.random.default_rng(0).normal(size=(model.window, 9)).astype(np.float32)
    model(x)
    t0 = time.perf_counter()
    for _ in range(n):
        model(x)
    ms = 1e3 * (time.perf_counter() - t0) / n
    print(f'inference: {ms:.2f} ms per window (W={model.window}); at 50 Hz that is {ms * 5:.1f} % of one core')


def parity(model_path):
    import sys
    import torch
    sys.path.insert(0, os.path.join(HERE, '..', 'analysis'))
    from ablation import JointStateCNN as TorchCNN
    z = np.load(model_path)
    ch = z['channels']
    net = TorchCNN(len(ch), int(z['window_size']))
    sd = {k: torch.tensor(z[k]) for k in net.state_dict() if k in z.files}
    net.load_state_dict(sd, strict=False)
    net.eval()
    model = JointStateCNN(model_path)
    # Inputs at the scale of the training data (mean + std * N(0, 1)) so that the contact
    # probabilities are not all saturated at 0 or 1.
    x = np.zeros((64, model.window, 9), np.float32)
    x[:, :, ch] = z['mean'] + z['std'] * np.random.default_rng(1).normal(size=(64, model.window, len(ch)))
    p_np, i_np, m_np = model(x)
    with torch.no_grad():
        out = net(torch.tensor((x[:, :, ch] - z['mean']) / z['std'], dtype=torch.float32))
    err = max(np.abs(p_np - out['contact'][:, 0].numpy()).max(), np.abs(i_np - out['incline'].numpy()).max(),
              np.abs(m_np - out['material'].numpy()).max())
    assert err < 1e-3, err
    print(f'parity with PyTorch: max abs difference {err:.2e} '
          f'(contact p from {p_np.min():.2f} to {p_np.max():.2f} over {len(x)} windows)')


def load_recording(folder):
    js = np.loadtxt(os.path.join(folder, 'joint_states.csv'), delimiter=',', skiprows=1)
    js = js[np.argsort(js[:, 0], kind='stable')]
    ct = np.loadtxt(os.path.join(folder, 'contact.csv'), delimiter=',', skiprows=1)
    ct = ct[np.argsort(ct[:, 0], kind='stable')]
    label = np.interp(js[:, 0], ct[:, 0], ct[:, 1]) > 0.5   # same rule as the training labels
    return js[:, 0], js[:, 1:10].astype(np.float32), label


def intervals(t, label):
    d = np.diff(label.astype(int))
    on = list(np.where(d == 1)[0] + 1)
    off = list(np.where(d == -1)[0] + 1)
    if label[0]:
        on = [0] + on
    if label[-1]:
        off = off + [len(label) - 1]
    return [(t[a], t[b]) for a, b in zip(on, off)]


def replay(model, data_dir, part, rate_hz, hyst_kw, batch=256):
    folders = sorted(os.path.dirname(p) for p in glob.glob(os.path.join(data_dir, '*', '*', 'joint_states.csv')))
    step = int(round(1000 / rate_hz))
    hits = misses = false_edges = repeats = 0
    lat, idle_s = [], 0.0
    per = []
    for folder in folders:
        t, x, label = load_recording(folder)
        start = int(0.8 * len(t)) if part == 'val' else 0
        e0 = start + model.window   # first window that lies fully in the evaluated part
        # Run the 50 Hz detector from the start of the recording so that the hysteresis state
        # at e0 is warm (as on the robot); count only edges at or after e0.
        ends = np.arange(model.window + (e0 - model.window) % step, len(t) + 1, step)
        p = np.concatenate([model(np.stack([x[e - model.window:e] for e in ends[i:i + batch]]))[0]
                            for i in range(0, len(ends), batch)])
        h = Hysteresis(**hyst_kw)
        seg_t, seg_l = t[e0 - 1:], label[e0 - 1:]
        edges = [t[e - 1] for e, pi in zip(ends, p) if h.update(pi)[1] and t[e - 1] >= seg_t[0]]
        all_ivs = intervals(seg_t, seg_l)
        # An onset counts only if the label was 0 just before it (true touchdown, not the segment start).
        ivs = [iv for iv in all_ivs if iv[0] > seg_t[0] + 0.05]
        f_hits = 0
        for a, b in ivs:
            cand = [e for e in edges if a - 0.1 <= e <= b]
            if cand:
                hits += 1
                f_hits += 1
                lat.append(cand[0] - a)
                repeats += len(cand) - 1    # extra rising edges during the same contact
            else:
                misses += 1
        # Edges up to 0.2 s after a contact ends are late detections of that contact, not false.
        fe = [e for e in edges if not any(a - 0.1 <= e <= b + 0.2 for a, b in all_ivs)]
        false_edges += len(fe)
        idle_s += float(np.sum(np.diff(seg_t)[~seg_l[1:]]))
        per.append((os.path.relpath(folder, data_dir), f_hits, len(ivs), len(fe)))
    lat = np.array(lat)
    med, p90 = (1e3 * np.median(lat), 1e3 * np.percentile(lat, 90)) if len(lat) else (np.nan, np.nan)
    print(f'replay ({part}): {hits}/{hits + misses} touchdowns detected, '
          f'latency median {med:.0f} ms, p90 {p90:.0f} ms; {repeats} repeat edges during a contact; '
          f'{false_edges} false triggers in {idle_s / 60:.1f} min without contact')
    worst = sorted(per, key=lambda r: (r[1] - r[2], -r[3]))[:5]
    print('  weakest recordings (name, detected, touchdowns, false):', worst)
    return dict(hits=hits, touchdowns=hits + misses, latency_ms_median=float(med), latency_ms_p90=float(p90),
                repeats=repeats, false_triggers=false_edges, idle_minutes=idle_s / 60)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.path.join(HERE, 'implicit_classifier.npz'))
    ap.add_argument('--data', help='tactile_data folder for the replay check')
    ap.add_argument('--part', choices=['val', 'all'], default='val')
    ap.add_argument('--rate-hz', type=float, default=50.0)
    ap.add_argument('--p-on', type=float, default=0.7)
    ap.add_argument('--p-off', type=float, default=0.3)
    ap.add_argument('--n-on', type=int, default=3)
    ap.add_argument('--n-off', type=int, default=5)
    ap.add_argument('--parity', action='store_true')
    args = ap.parse_args()

    test_state_machine()
    model = JointStateCNN(args.model)
    bench(model)
    buf = JointStateBuffer(4)
    for i in range(6):
        buf.push([i] * 3, [i] * 3, [i] * 3)
    assert buf.full() and buf.window()[:, 0].tolist() == [2, 3, 4, 5]
    if args.parity:
        parity(args.model)
    if args.data:
        replay(model, os.path.expanduser(args.data), args.part, args.rate_hz,
               dict(p_on=args.p_on, p_off=args.p_off, n_on=args.n_on, n_off=args.n_off))


if __name__ == '__main__':
    main()
