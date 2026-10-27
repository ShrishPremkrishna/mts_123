"""One fixed validation-only ablation: frozen ImageNet features and linear heads.

This command never constructs a test dataset. Keep its result exploratory until a
fresh recording holdout is available; the original test has already been observed.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
import numpy as np
import torch
from torch.utils.data import DataLoader
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from torchvision.models import resnet50, ResNet50_Weights
from .data import sha256
from .train import ImageDataset, choose_device, classification_metrics


@torch.inference_mode()
def features(backbone, dataset, device, batch_size):
    x_values, y_values = [], []
    for step, (x, y) in enumerate(DataLoader(dataset, batch_size, shuffle=False), 1):
        x_values.append(backbone(x.to(device)).cpu().numpy())
        y_values.append(y.numpy())
        if step % 20 == 0:
            print(f"Extracted {sum(len(a) for a in y_values)}/{len(dataset)} images", flush=True)
    return np.concatenate(x_values), np.concatenate(y_values)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("manifest", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--root", type=Path)
    p.add_argument("--device", default="auto")
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--threads", type=int, default=4)
    args = p.parse_args()
    if args.out.exists():
        raise FileExistsError("Preserve previous results; choose a new ablation directory")
    args.out.mkdir(parents=True)
    torch.set_num_threads(args.threads)
    torch.manual_seed(123)
    device = choose_device(args.device)
    weights = ResNet50_Weights.DEFAULT
    backbone = resnet50(weights=weights).to(device).eval()
    backbone.fc = torch.nn.Identity()
    started = time.monotonic()
    values = {}
    for split in ("train", "val"):
        data = ImageDataset(args.manifest, split, args.root)
        values[split] = features(backbone, data, device, args.batch_size)
        np.savez_compressed(args.out / f"{split}_features.npz", x=values[split][0], y=values[split][1])
    train_x, train_y = values["train"]
    val_x, val_y = values["val"]
    scaler = StandardScaler().fit(train_x)
    train_x, val_x = scaler.transform(train_x), scaler.transform(val_x)
    metrics = {}
    arrays = dict(feature_mean=scaler.mean_, feature_scale=scaler.scale_)
    for j, name in ((1, "incline"), (2, "material")):
        head = LogisticRegression(C=1.0, class_weight="balanced", max_iter=2000, random_state=123)
        head.fit(train_x, train_y[:, j])
        metrics[name] = classification_metrics(val_y[:, j], head.predict(val_x), 6)
        arrays.update({name + "_coef": head.coef_, name + "_intercept": head.intercept_, name + "_classes": head.classes_})
    np.savez_compressed(args.out / "linear_heads.npz", **arrays)
    report = dict(manifest_sha256=sha256(args.manifest), source_sha256={f.name:sha256(f) for f in Path(__file__).parent.glob("*.py")},
                  device=str(device), torch_version=str(torch.__version__), weights=str(weights),
                  training_samples=len(train_y), validation_samples=len(val_y), seed=123,
                  classifier="standardized frozen ResNet50 features + balanced L2 multinomial logistic heads; C=1; max_iter=2000",
                  metrics=metrics, elapsed_s=time.monotonic()-started, test_accessed=False,
                  scope="single fixed validation-only diagnostic; no tuning, test result, or robot deployment claim")
    (args.out / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
