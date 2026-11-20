"""Learned action-noise std (actor `distribution.std_param`) per joint, read from checkpoints (CPU, no sim).

Also checks that each exported policy.json first layer equals the checkpoint's first layer (so the
deployed network is the trained one), and records the obs-normalizer sample count.

  python policy_std.py --out OUT.json
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

RUNS = Path.home() / "runs" / "opt-lab-2"
CKPTS = {
  "fasxqy4z model_2999 (good transfer)": RUNS / "ext-fasxqy4z/model_2999.pt",
  "Part 1 parent 1c5r0n8c model_2999": RUNS / "rp-p1-gallop_c-s42/model_2999.pt",
  "Part 2 final P wmhcp24q model_4496": RUNS / "rp-p2-s42-r2-cont-1500/model_4496.pt",
  "Bumpy +500 model_4995": RUNS / "rp-p3-final-bumpyft-kpwide-leg1/model_4995.pt",
  "Bumpy +2000 23pt1499 model_6492 (jerky)": RUNS / "rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt",
  "Flat ctrl +2000 13m4vcia model_6492": RUNS / "rp-p3-final-flatctrl-kpwide-leg4/model_6492.pt",
}
JOINTS = ["FR1", "FR2", "FR3", "FL1", "FL2", "FL3", "BR1", "BR2", "BR3", "BL1", "BL2", "BL3"]

if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--out", required=True)
  a = ap.parse_args()
  res = {}
  for name, p in CKPTS.items():
    if not p.exists():
      res[name] = {"missing": str(p)}
      continue
    d = torch.load(p, map_location="cpu", weights_only=False)
    sd = d["actor_state_dict"]
    std = sd["distribution.std_param"].numpy()
    w0 = [v for k, v in sd.items() if k.endswith("mlp.0.weight")][0].numpy()
    pj = p.parent / "policy.json"
    match = None
    if pj.exists():
      L0 = np.asarray(json.loads(pj.read_text())["layers"][0]["weights"][0]).T  # (256, 960)
      # The export folds the obs normalizer into layer 0 and stores history newest-first, while the
      # training obs is oldest-first. Correlate against the folded, frame-reversed checkpoint layer.
      folded = w0 / np.maximum(sd["obs_normalizer._std"].numpy()[0], 1e-8)
      rev = folded.reshape(w0.shape[0], 20, 48)[:, ::-1].reshape(w0.shape[0], -1)
      corr = np.corrcoef(L0.ravel(), rev.ravel())[0, 1]
      match = float(corr)
    res[name] = dict(
      checkpoint=str(p), iter=int(d.get("iter", -1)),
      std_mean=float(std.mean()), std_min=float(std.min()), std_max=float(std.max()),
      std_per_joint=dict(zip(JOINTS, [round(float(x), 3) for x in std])),
      obs_normalizer_count=float(sd["obs_normalizer.count"]),
      export_layer0_corr_frames_reversed=match,
    )
    print(name, round(float(std.mean()), 3), [round(float(x), 2) for x in std], "export corr", match)
  Path(a.out).write_text(json.dumps(res, indent=1) + "\n")
