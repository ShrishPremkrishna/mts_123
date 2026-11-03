"""Retrain the explicit (DenseTact image) classifier under a leakage-free split.

Uses the contact-filtered image list that notebooks/explicit_classifier.ipynb writes
(lookups/explicit_train_dataset.csv) and the notebook's model, optimizer and schedule
(ResNet-50, two linear heads, AdamW lr 1e-3 wrapped in PCGrad, batch 32, 10 epochs).
Only the split changes:

* blocked : per recording, the last 20 % of contact images (by capture time) is validation.
* session : train on the first collection set, validate on the second ("*_new", 15-75 deg).
* random  : the notebook's seeded 90/10 split, for reference.

    python analysis/explicit_split.py --data ~/tactile_data --split blocked --out results/explicit_blocked.json
"""

import argparse
import json
import os
import random
import re
import time

import numpy as np
import pandas as pd
import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision import models, transforms

from ablation import PCGrad

TRANSFORM = transforms.Compose([
    transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])


class Images(Dataset):
    def __init__(self, df):
        self.df = df.reset_index(drop=True)

    def __len__(self):
        return len(self.df)

    def __getitem__(self, i):
        r = self.df.iloc[i]
        return TRANSFORM(Image.open(r.image_path).convert('RGB')), int(r.incline_label), int(r.material_label)


class MultiTask(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.base = models.resnet50(weights=models.ResNet50_Weights.IMAGENET1K_V1)
        n = self.base.fc.in_features
        self.base.fc = torch.nn.Identity()
        self.incline_head = torch.nn.Linear(n, 6)
        self.material_head = torch.nn.Linear(n, 6)

    def forward(self, x):
        f = self.base(x)
        return self.incline_head(f), self.material_head(f)


def split(df, data_dir, how, seed=0):
    rel = df.image_path.map(lambda p: os.path.relpath(p, data_dir))
    df = df.assign(folder=rel.map(lambda r: os.path.dirname(os.path.dirname(r))),
                   t=rel.map(lambda r: float(re.search(r'img_([0-9.]+)\.jpe?g', r).group(1))))
    if how == 'random':
        perm = torch.randperm(len(df), generator=torch.Generator().manual_seed(seed)).numpy()
        val = np.zeros(len(df), bool)
        val[perm[int(0.9 * len(df)):]] = True
    elif how == 'blocked':
        rank = df.groupby('folder').t.rank(pct=True)
        val = (rank > 0.8).values
    else:
        top = df.folder.map(lambda f: f.split(os.sep)[0])
        val = (top.str.endswith('degrees_new') & ~top.str.startswith('0 ')).values
    return df[~val], df[val]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--split', choices=['random', 'blocked', 'session'], default='blocked')
    ap.add_argument('--epochs', type=int, default=10)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='mps' if torch.backends.mps.is_available() else 'cpu')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    dev = torch.device(args.device)
    df = pd.read_csv(os.path.join(args.data, 'lookups', 'explicit_train_dataset.csv'))
    # Image paths in the CSV are absolute; --data must be the same data_dir the notebook used.
    tr, va = split(df, args.data, args.split, args.seed)
    print(f'{args.split}: train {len(tr)} val {len(va)}', flush=True)
    loaders = {'train': DataLoader(Images(tr), batch_size=32, shuffle=True),
               'val': DataLoader(Images(va), batch_size=32, shuffle=False)}
    model = MultiTask().to(dev)
    ce = torch.nn.CrossEntropyLoss()
    opt = PCGrad(torch.optim.AdamW(model.parameters(), lr=1e-3))
    history, t0 = [], time.time()
    for epoch in range(1, args.epochs + 1):
        row = dict(epoch=epoch)
        for phase in ('train', 'val'):
            model.train(phase == 'train')
            ok_i = ok_m = n = 0
            for x, yi, ym in loaders[phase]:
                x, yi, ym = x.to(dev), yi.to(dev), ym.to(dev)
                with torch.set_grad_enabled(phase == 'train'):
                    oi, om = model(x)
                    if phase == 'train':
                        opt.pc_backward([ce(oi, yi), ce(om, ym)])
                        opt.step()
                ok_i += (oi.argmax(1) == yi).sum().item()
                ok_m += (om.argmax(1) == ym).sum().item()
                n += len(yi)
            row[phase] = dict(incline_acc=ok_i / n, material_acc=ok_m / n, n=n)
        history.append(row)
        print(f"ep {epoch} val incline {row['val']['incline_acc']:.4f} material {row['val']['material_acc']:.4f} "
              f"| train incline {row['train']['incline_acc']:.4f} material {row['train']['material_acc']:.4f} "
              f"t {time.time() - t0:.0f}s", flush=True)
    res = dict(args=vars(args), final=history[-1], history=history, seconds=time.time() - t0,
               best_val={k: max(h['val'][k] for h in history) for k in ('incline_acc', 'material_acc')})
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
