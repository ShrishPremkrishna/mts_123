"""Train the implicit (joint-state) classifier under controlled ablations.

The model, labels, loss masking, contact-weight decay and PCGrad are the same as
in notebooks/implicit_classifier.ipynb. This script adds three things the
notebook cannot show:

* --split blocked : per recording, the last 20 % of samples is validation and a
  one-window gap separates it from training. The notebook's random split puts
  windows that share up to (W - 20)/W of their samples into both sets.
* --split session : train on the first collection set, validate on the second
  ("*_new" folders, 15-75 deg); a different-day test.
* --channels      : position / velocity / effort subsets.
* --label last    : label a window by the contact state of its newest sample instead of the
  notebook's majority vote. A majority label switches only when half the window is in
  contact (about W/2 ms of delay); the newest-sample label suits a real-time detector.

Example:
    python analysis/ablation.py --data ~/tactile_data --window 200 \
        --split blocked --channels eff --epochs 30 --out results/w200_blocked_eff.json
"""

import argparse
import json
import os
import random
import time

import numpy as np
import pandas as pd
import torch
from scipy.interpolate import interp1d

INCLINE_CLASSES = [15, 30, 45, 60, 75, 0]
MATERIAL_CLASSES = ['none', 'felt pad', 'rough', 'slip', 'very rough', 'foam']
JOINT_COLUMNS = [f'leg_front_r_{j}_{q}' for q in ('position', 'velocity', 'effort') for j in (1, 2, 3)]
CHANNELS = {
    'all': list(range(9)),
    'pos': [0, 1, 2], 'vel': [3, 4, 5], 'eff': [6, 7, 8],
    'pos_vel': [0, 1, 2, 3, 4, 5], 'pos_eff': [0, 1, 2, 6, 7, 8], 'vel_eff': [3, 4, 5, 6, 7, 8],
}


def parse_folder_name(rel_path):
    """Notebook label parser, applied to the path relative to the data root."""
    s = rel_path.lower()
    incline = next((i for i, a in enumerate(INCLINE_CLASSES) if str(a) in s), None)
    material = next((i for i, m in enumerate(MATERIAL_CLASSES) if m in s), None)
    if 'felt_pad' in s or 'felt pad' in s:
        material = MATERIAL_CLASSES.index('felt pad')
    elif 'very_rough' in s or 'very rough' in s or 'very-rough' in s:
        material = MATERIAL_CLASSES.index('very rough')
    return incline, material


def load_recordings(root):
    """Return one dict per recording folder: sorted motor states and per-sample contact."""
    recs = []
    for f1 in sorted(os.listdir(root)):
        if f1.startswith('.') or f1 in ('reference_images', 'lookups'):
            continue
        for f2 in sorted(os.listdir(os.path.join(root, f1))):
            path = os.path.join(root, f1, f2)
            if f2.startswith('.') or not os.path.isdir(path):
                continue
            rel = os.path.join(f1, f2)
            incline, material = parse_folder_name(rel)
            js = pd.read_csv(os.path.join(path, 'joint_states.csv')).sort_values('timestamp')
            ct = pd.read_csv(os.path.join(path, 'contact.csv')).sort_values('timestamp')
            t = js['timestamp'].values
            # Same as the notebook: linear interpolation of the 10 Hz label, then > 0.5.
            f = interp1d(ct['timestamp'], ct['contact_value'], bounds_error=False, fill_value='extrapolate')
            contact = (f(t) > 0.5).astype(np.float32)
            recs.append(dict(rel=rel, incline=incline, material=material, t=t,
                             x=js[JOINT_COLUMNS].values.astype(np.float32), contact=contact,
                             session=2 if ('_new' in f1) else 1))
    return recs


def make_windows(recs, window, stride, split, label='majority', val_frac=0.2, seed=0):
    """Concatenate recordings and return start indices and labels for train and val windows."""
    xs, cs, starts, y_c, y_i, y_m, is_val = [], [], [], [], [], [], []
    offset = 0
    for r in recs:
        n = len(r['x'])
        xs.append(r['x'])
        cs.append(r['contact'])
        csum = np.concatenate([[0.0], np.cumsum(r['contact'])])
        s = np.arange(0, n - window + 1, stride)
        frac = (csum[s + window] - csum[s]) / window
        if split == 'blocked':
            cut = int(n * (1 - val_frac))
            val = s >= cut
            keep = val | (s + window <= cut - window)  # one-window gap before the cut
        elif split == 'session':
            angled = r['incline'] != INCLINE_CLASSES.index(0) and r['rel'].split(os.sep)[0].endswith(('degrees', 'degrees_new'))
            val = np.full(len(s), r['session'] == 2 and angled)
            keep = np.ones(len(s), bool)
        else:
            val = np.zeros(len(s), bool)
            keep = np.ones(len(s), bool)
        starts.append(s[keep] + offset)
        lab = frac > 0.5 if label == 'majority' else r['contact'][s + window - 1] > 0.5
        y_c.append(lab[keep].astype(np.float32))
        y_i.append(np.full(keep.sum(), r['incline']))
        y_m.append(np.full(keep.sum(), r['material']))
        is_val.append(val[keep])
        offset += n
    X = np.concatenate(xs)
    starts, y_c, y_i, y_m, is_val = map(np.concatenate, (starts, y_c, y_i, y_m, is_val))
    if split == 'random':  # torch.utils.data.random_split equivalent, 90/10
        g = torch.Generator().manual_seed(seed)
        perm = torch.randperm(len(starts), generator=g).numpy()
        is_val = np.zeros(len(starts), bool)
        is_val[perm[int(0.9 * len(starts)):]] = True
    return X, starts, y_c, y_i, y_m, is_val


class JointStateCNN(torch.nn.Module):
    """Same layers and names as the notebook model (the robot exporter relies on the names)."""

    def __init__(self, input_channels=9, window_size=100):
        super().__init__()
        self.conv1 = torch.nn.Conv1d(input_channels, 32, kernel_size=5, padding=2)
        self.bn1 = torch.nn.BatchNorm1d(32)
        self.conv2 = torch.nn.Conv1d(32, 64, kernel_size=5, padding=2)
        self.bn2 = torch.nn.BatchNorm1d(64)
        self.conv3 = torch.nn.Conv1d(64, 128, kernel_size=5, padding=2)
        self.bn3 = torch.nn.BatchNorm1d(128)
        self.pool = torch.nn.MaxPool1d(2)
        output_size = window_size // 2 // 2 // 2
        self.fc1 = torch.nn.Linear(128 * output_size, 256)
        self.dropout = torch.nn.Dropout(0.5)
        self.contact_head = torch.nn.Linear(256, 1)
        self.incline_head = torch.nn.Linear(256, len(INCLINE_CLASSES))
        self.material_head = torch.nn.Linear(256, len(MATERIAL_CLASSES))

    def forward(self, x):
        x = x.transpose(1, 2)
        x = self.pool(torch.relu(self.bn1(self.conv1(x))))
        x = self.pool(torch.relu(self.bn2(self.conv2(x))))
        x = self.pool(torch.relu(self.bn3(self.conv3(x))))
        x = x.reshape(x.size(0), -1)
        x = self.dropout(torch.relu(self.fc1(x)))
        return {'contact': torch.sigmoid(self.contact_head(x)),
                'incline': self.incline_head(x),
                'material': self.material_head(x)}


class PCGrad:
    """Projected Conflicting Gradient (Yu et al., 2020), as completed in the notebooks."""

    def __init__(self, optimizer, reduction='mean'):
        self._optim, self._reduction = optimizer, reduction

    def zero_grad(self):
        self._optim.zero_grad(set_to_none=True)

    def step(self):
        self._optim.step()

    def pc_backward(self, objectives):
        grads, shapes, has_grads = [], [], []
        for obj in objectives:
            self._optim.zero_grad(set_to_none=True)
            obj.backward(retain_graph=True)
            g, s, h = [], [], []
            for group in self._optim.param_groups:
                for p in group['params']:
                    s.append(p.shape)
                    g.append(torch.zeros_like(p) if p.grad is None else p.grad.clone())
                    h.append(torch.zeros_like(p) if p.grad is None else torch.ones_like(p))
            grads.append(torch.cat([x.flatten() for x in g]))
            has_grads.append(torch.cat([x.flatten() for x in h]))
            shapes.append(s)
        shared = torch.stack(has_grads).prod(0).bool()
        pc_grad = [g.clone() for g in grads]
        for g_i in pc_grad:
            random.shuffle(grads)
            for g_j in grads:
                dot = torch.dot(g_i, g_j)
                if dot < 0:
                    g_i -= dot * g_j / (g_j.norm() ** 2)
        merged = torch.zeros_like(grads[0])
        stack = torch.stack(pc_grad)
        merged[shared] = stack[:, shared].mean(0) if self._reduction == 'mean' else stack[:, shared].sum(0)
        merged[~shared] = stack[:, ~shared].sum(0)
        idx = 0
        for group in self._optim.param_groups:
            for p, shape in zip(group['params'], shapes[0]):
                n = int(np.prod(shape))
                p.grad = merged[idx:idx + n].view(shape).clone()
                idx += n


def evaluate_batch(out, y_c, y_i, y_m):
    pred_c = (out['contact'].squeeze(1) > 0.5).float()
    m = y_c.bool()
    return dict(c_ok=(pred_c == y_c).sum().item(), n=len(y_c),
                tp=((pred_c == 1) & m).sum().item(), fp=((pred_c == 1) & ~m).sum().item(),
                fn=((pred_c == 0) & m).sum().item(),
                i_ok=(out['incline'].argmax(1)[m] == y_i[m]).sum().item(),
                m_ok=(out['material'].argmax(1)[m] == y_m[m]).sum().item(), n_contact=m.sum().item())


def summarize(acc):
    tot = {k: sum(a[k] for a in acc) for k in acc[0]}
    prec = tot['tp'] / max(1, tot['tp'] + tot['fp'])
    rec = tot['tp'] / max(1, tot['tp'] + tot['fn'])
    return dict(contact_acc=tot['c_ok'] / tot['n'], contact_precision=prec, contact_recall=rec,
                incline_acc=tot['i_ok'] / max(1, tot['n_contact']),
                material_acc=tot['m_ok'] / max(1, tot['n_contact']), n=tot['n'], n_contact=tot['n_contact'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--window', type=int, default=200)
    ap.add_argument('--stride', type=int, default=20)
    ap.add_argument('--split', choices=['random', 'blocked', 'session'], default='random')
    ap.add_argument('--channels', choices=list(CHANNELS), default='all')
    ap.add_argument('--label', choices=['majority', 'last'], default='majority')
    ap.add_argument('--norm', action='store_true', help='standardize channels with train-set mean and std')
    ap.add_argument('--epochs', type=int, default=50)
    ap.add_argument('--batch', type=int, default=512)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='mps' if torch.backends.mps.is_available() else 'cpu')
    ap.add_argument('--out', required=True, help='JSON results file')
    ap.add_argument('--save-model', help='write the trained model as a robot .npz')
    args = ap.parse_args()

    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    dev = torch.device(args.device)
    t0 = time.time()
    recs = load_recordings(args.data)
    X, starts, y_c, y_i, y_m, is_val = make_windows(recs, args.window, args.stride, args.split, args.label, seed=args.seed)
    ch = CHANNELS[args.channels]
    X = X[:, ch]
    tr = ~is_val
    if args.norm:
        rows = (starts[tr][:, None] + np.arange(0, args.window, 10)).ravel()
        mean, std = X[rows].mean(0), X[rows].std(0) + 1e-6
    else:
        mean, std = np.zeros(len(ch), np.float32), np.ones(len(ch), np.float32)
    Xd = torch.tensor((X - mean) / std, dtype=torch.float32, device=dev)
    ar = torch.arange(args.window, device=dev)
    sets = {k: dict(s=torch.tensor(starts[m], device=dev), c=torch.tensor(y_c[m], device=dev),
                    i=torch.tensor(y_i[m], device=dev), m=torch.tensor(y_m[m], device=dev))
            for k, m in (('train', tr), ('val', is_val))}
    print(f"windows train {len(sets['train']['s'])} val {len(sets['val']['s'])} "
          f"(contact frac {y_c[tr].mean():.3f}/{y_c[is_val].mean():.3f}) load {time.time() - t0:.0f}s", flush=True)

    model = JointStateCNN(len(ch), args.window).to(dev)
    bce, ce = torch.nn.BCELoss(), torch.nn.CrossEntropyLoss(reduction='none')
    opt = PCGrad(torch.optim.Adam(model.parameters(), lr=args.lr))
    history = []
    for epoch in range(1, args.epochs + 1):
        cw = max(0.1, 1.0 - (epoch - 1) / max(1, args.epochs - 1))
        row = dict(epoch=epoch)
        for phase in ('train', 'val'):
            d = sets[phase]
            model.train(phase == 'train')
            order = torch.randperm(len(d['s']), device=dev) if phase == 'train' else torch.arange(len(d['s']), device=dev)
            acc = []
            for b in range(0, len(order), args.batch):
                idx = order[b:b + args.batch]
                x = Xd[d['s'][idx][:, None] + ar]
                yc, yi, ym = d['c'][idx], d['i'][idx], d['m'][idx]
                with torch.set_grad_enabled(phase == 'train'):
                    out = model(x)
                    if phase == 'train':
                        mask = yc
                        objectives = [cw * bce(out['contact'].squeeze(1), yc)]
                        if mask.any():
                            objectives.append((ce(out['incline'], yi) * mask).mean())
                            objectives.append((ce(out['material'], ym) * mask).mean())
                        opt.pc_backward(objectives)
                        opt.step()
                acc.append(evaluate_batch(out, yc, yi, ym))
            row[phase] = summarize(acc)
        history.append(row)
        v = row['val']
        print(f"ep {epoch:3d} val contact {v['contact_acc']:.4f} incline {v['incline_acc']:.4f} "
              f"material {v['material_acc']:.4f} | train contact {row['train']['contact_acc']:.4f} "
              f"t {time.time() - t0:.0f}s", flush=True)

    res = dict(args=vars(args), final=history[-1], history=history, seconds=time.time() - t0,
               best_val={k: max(h['val'][k] for h in history) for k in ('contact_acc', 'incline_acc', 'material_acc')})
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=1)
    if args.save_model:
        sd = {k: v.detach().cpu().numpy() for k, v in model.state_dict().items() if 'num_batches' not in k}
        np.savez(args.save_model, **sd, channels=np.array(ch), mean=mean, std=std,
                 window_size=args.window, sample_rate_hz=1000.0, label=args.label,
                 joint_columns=np.array(JOINT_COLUMNS)[ch],
                 incline_classes=np.array(INCLINE_CLASSES), material_classes=np.array(MATERIAL_CLASSES))


if __name__ == '__main__':
    main()
