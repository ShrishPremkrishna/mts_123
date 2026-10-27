"""Describe split coverage and label timing without modifying data or fitting models."""
from __future__ import annotations
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import numpy as np
from PIL import Image
from .data import read_csv, sha256


def split_coverage(manifest):
    train_conditions = {(r["incline"], r["material"]) for r in manifest["records"] if r["split"] == "train"}
    output = {}
    for split in ("train", "val", "test"):
        records = [r for r in manifest["records"] if r["split"] == split]
        images = [i for i in manifest["images"] if i["split"] == split]
        output[split] = dict(recordings=len(records), selected_images=len(images), classes={})
        for field, labels in (("incline", manifest["inclines"]), ("material", manifest["materials"])):
            output[split]["classes"][field] = [dict(label=label, recordings=sum(r[field] == k for r in records),
                images=sum(i[field] == k for i in images)) for k, label in enumerate(labels)]
        output[split]["conditions_absent_from_training"] = [r["id"] for r in records
            if (r["incline"], r["material"]) not in train_conditions]
    return output


def past_labels(contact, times):
    indices = np.searchsorted(contact[:, 0], times, side="right") - 1
    clipped = np.clip(indices, 0, len(contact) - 1)
    age = times - contact[clipped, 0]
    valid = (indices >= 0) & (times <= contact[-1, 0]) & (age <= .25)
    return contact[clipped, 1] > .5, valid, age


def audit(manifest_path, root=None):
    manifest_path = Path(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    root = Path(root or manifest["root"])
    digests = defaultdict(set)
    sizes = defaultdict(int)
    for item in manifest["images"]:
        path = root / item["path"]
        digests[sha256(path)].add(item["split"])
        with Image.open(path) as im:
            sizes[f"{im.width}x{im.height}"] += 1
    timing = {}
    # Test set contributes only split/duplicate metadata, never label-shift tuning.
    for split in ("train", "val"):
        ages, gaps, frame_gaps = [], [], []
        sensitivity = {str(delta): dict(comparable=0, changes=0) for delta in (-.1, -.05, .05, .1)}
        for record in manifest["records"]:
            if record["split"] != split:
                continue
            folder = root / record["id"]
            contact = read_csv(folder / "contact.csv")
            with (folder / "image_index.csv").open() as f:
                timestamps = np.asarray([float(r["timestamp"]) for r in csv.DictReader(f)])
            labels, valid, age = past_labels(contact, timestamps)
            ages.extend(age[valid]); gaps.extend(np.diff(contact[:, 0])); frame_gaps.extend(np.diff(timestamps))
            for delta, result in sensitivity.items():
                shifted, ok, _ = past_labels(contact, timestamps + float(delta))
                compare = valid & ok
                result["comparable"] += int(compare.sum())
                result["changes"] += int((labels[compare] != shifted[compare]).sum())
        quantiles = lambda values: dict(zip(("min", "median", "p95", "max"), np.quantile(values, [0,.5,.95,1]).tolist()))
        timing[split] = dict(label_age_s=quantiles(ages), annotation_interval_s=quantiles(gaps),
                            image_interval_s=quantiles(frame_gaps), offset_sensitivity=sensitivity)
    return dict(manifest_sha256=sha256(manifest_path), seed=manifest["seed"], group_by=manifest["group_by"],
                coverage=split_coverage(manifest), selected_image_sizes=dict(sizes),
                exact_image_duplicate_hashes_crossing_splits=sum(len(splits) > 1 for splits in digests.values()),
                train_val_timing=timing,
                caveats=["No near-duplicate detection; frame correlations within recordings are expected.",
                         "Recorded contact values are detector outputs per the course handout, not human ground truth.",
                         "Offset sensitivity is descriptive only; no offset has been optimized or applied.",
                         "The eight test recordings do not cover every class; window/frame counts are not independent trials."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--root", type=Path)
    args = parser.parse_args()
    report = audit(args.manifest, args.root)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
