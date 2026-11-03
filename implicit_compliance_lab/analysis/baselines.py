"""Non-deep baselines for contact from joint states (blocked split, majority window labels).

Each window (W samples, stride 20) is reduced to per-channel means; a one-feature
threshold or a logistic regression then predicts contact. If a "the leg is not moving"
rule already scores high, the CNN's contact accuracy partly reflects the data-collection
protocol (the leg was paused for 1 s right after every detected contact).

    python analysis/baselines.py --data ~/tactile_data --window 200 --out results/baselines_w200.json
"""

import argparse
import json

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from ablation import load_recordings, make_windows


def window_means(X, starts, window, fn=lambda a: a):
    cs = np.concatenate([np.zeros((1, X.shape[1])), np.cumsum(fn(X).astype(np.float64), axis=0)])
    return ((cs[starts + window] - cs[starts]) / window).astype(np.float32)


def best_threshold(f_tr, y_tr, f_va, y_va):
    """Pick the threshold and direction on train, report val accuracy."""
    qs = np.quantile(f_tr, np.linspace(0.01, 0.99, 197))
    best = max(((np.mean((f_tr <= q) == y_tr) if d else np.mean((f_tr > q) == y_tr), q, d)
                for q in qs for d in (True, False)))
    _, q, d = best
    pred = f_va <= q if d else f_va > q
    return float(np.mean(pred == y_va)), float(q), 'below' if d else 'above'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--window', type=int, default=200)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    X, starts, y_c, _, _, is_val = make_windows(load_recordings(args.data), args.window, 20, 'blocked')
    y = y_c.astype(bool)
    tr, va = ~is_val, is_val
    mean = window_means(X, starts, args.window)
    absmean = window_means(X, starts, args.window, np.abs)
    speed = np.linalg.norm(absmean[:, 3:6], axis=1)
    res = {'window': args.window, 'n_train': int(tr.sum()), 'n_val': int(va.sum()),
           'majority_class_val': float(max(y[va].mean(), 1 - y[va].mean()))}
    acc, q, d = best_threshold(speed[tr], y[tr], speed[va], y[va])
    res['threshold_mean_abs_velocity'] = dict(val_acc=acc, threshold=q, contact_when=d)
    feats = {
        'logreg_pos': mean[:, 0:3],
        'logreg_abs_vel': absmean[:, 3:6],
        'logreg_eff': mean[:, 6:9],
        'logreg_vel_eff': np.hstack([absmean[:, 3:6], mean[:, 6:9]]),
        'logreg_all': np.hstack([mean[:, 0:3], absmean[:, 3:6], mean[:, 6:9]]),
    }
    for name, F in feats.items():
        clf = make_pipeline(StandardScaler(), LogisticRegression(max_iter=2000)).fit(F[tr], y[tr])
        res[name] = dict(val_acc=float(clf.score(F[va], y[va])))
    print(json.dumps(res, indent=1))
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
