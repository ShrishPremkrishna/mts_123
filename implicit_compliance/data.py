"""Recording-disjoint preparation. Motor effort is torque-like effort, not acceleration."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path
import zipfile
import numpy as np

INCLINES = [0, 15, 30, 45, 60, 75]
MATERIALS = ["none", "felt_pad", "rough", "slip", "very_rough", "foam"]
CHANNELS = [f"leg_front_r_{j}_{kind}" for kind in ("position", "velocity", "effort") for j in (1, 2, 3)]
CONTACT_REFERENCES = ["image_20250305_005203.jpg", "image_20250305_005206.jpg", "image_20250305_005207.jpg", "image_20250305_005209.jpg", "image_20250305_005211.jpg"]
NO_CONTACT_REFERENCES = [f"image_20250304_23331{i}.jpg" for i in range(1, 6)]


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def extract_dataset(archive, destination):
    """Extract only ordinary paths, never traversal or symbolic links."""
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        for item in z.infolist():
            target = (destination / item.filename).resolve()
            if not target.is_relative_to(destination) or (item.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError(f"Unsafe archive entry: {item.filename}")
        z.extractall(destination)
    return destination / "tactile_data"


def labels_for(path):
    """Parse bounded angle tokens; 'very rough' must win over 'rough'."""
    text = str(path).lower().replace("-", "_").replace(" ", "_")
    angles = re.findall(r"(?<!\d)(75|60|45|30|15|0)(?!\d)", text)
    if not angles or len(set(angles)) != 1:
        raise ValueError(f"Ambiguous/missing incline: {path}")
    material = next((m for m in ("very_rough", "felt_pad", "foam", "slip", "rough", "none") if m in text), None)
    if material is None:
        raise ValueError(f"Missing material: {path}")
    return INCLINES.index(int(angles[0])), MATERIALS.index(material)


def read_csv(path, columns=None):
    with Path(path).open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        indices = list(range(len(header))) if columns is None else [header.index(c) for c in columns]
        values = np.array([[float(row[i]) for i in indices] for row in reader if row], dtype=np.float64)
    if values.ndim != 2 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError(f"Invalid or insufficient numerical data: {path}")
    values = values[np.argsort(values[:, 0], kind="stable")]
    # Keep the final measurement for duplicate timestamps; never average labels.
    keep = np.r_[np.diff(values[:, 0]) > 0, True]
    return values[keep]


def split_recordings(records, seed=123, group_by="recording"):
    """Split whole acquisitions/conditions before windows or images are created."""
    if group_by not in ("recording", "condition"):
        raise ValueError("group_by must be recording or condition")
    def key(r):
        return r["id"] if group_by == "recording" else f'{r["incline"]}:{r["material"]}'
    keys = sorted({key(r) for r in records}, key=lambda k: hashlib.sha256(f"{seed}:{k}".encode()).hexdigest())
    if len(keys) < 3:
        raise ValueError("At least three independent groups required for train/validation/test")
    n = max(1, round(len(keys) * .15))
    assignment = {k: "train" for k in keys}
    assignment.update({k: "val" for k in keys[:n]})
    assignment.update({k: "test" for k in keys[n:2*n]})
    # Guarantee training sees each category; do not repair using held-out samples.
    for field in ("incline", "material"):
        for label in sorted({r[field] for r in records}):
            if not any(r[field] == label and assignment[key(r)] == "train" for r in records):
                candidate = next(key(r) for r in sorted(records, key=lambda r: r["id"]) if r[field] == label)
                assignment[candidate] = "train"
    if any(not any(v == s for v in assignment.values()) for s in ("train", "val", "test")):
        raise ValueError("Insufficient groups for split with all categories represented in training")
    return {r["id"]: assignment[key(r)] for r in records}


def resample_recording(joints, contact, sample_hz=1000., max_gap_s=.025, max_label_age_s=.25):
    """Uniform physical-time grid; labels use latest past annotation, never extrapolation."""
    if sample_hz <= 0 or max_gap_s <= 0:
        raise ValueError("Sampling rate and maximum gap must be positive")
    start, end = max(joints[0, 0], contact[0, 0]), min(joints[-1, 0], contact[-1, 0])
    if end <= start:
        raise ValueError("Joint and contact timestamps do not overlap")
    t = start + np.arange(int(np.floor((end - start) * sample_hz)) + 1) / sample_hz
    x = np.stack([np.interp(t, joints[:, 0], joints[:, i]) for i in range(1, 10)], axis=1).astype(np.float32)
    right = np.clip(np.searchsorted(joints[:, 0], t, side="right"), 1, len(joints) - 1)
    valid = joints[right, 0] - joints[right - 1, 0] <= max_gap_s
    ci = np.clip(np.searchsorted(contact[:, 0], t, side="right") - 1, 0, len(contact) - 1)
    valid &= t - contact[ci, 0] <= max_label_age_s
    y = (contact[ci, 1:].max(axis=1) > .5).astype(np.int64)
    return t, x, y, valid


def valid_starts(valid, window, stride):
    if window < 2 or stride < 1:
        raise ValueError("window >= 2 and stride >= 1 required")
    starts = np.arange(0, max(0, len(valid) - window + 1), stride)
    prefix = np.r_[0, np.cumsum(~valid)]
    return starts[prefix[starts + window] == prefix[starts]]


def gray_image(path, size=224):
    from PIL import Image, ImageOps
    with Image.open(path) as image:
        # Same central square for reference frames and dataset frames.
        return np.asarray(ImageOps.fit(image.convert("L"), (size, size)), dtype=np.float32)


class ReferenceContact:
    """Course mean-absolute-difference contact filter, used only for explicit images."""
    def __init__(self, root):
        folder = Path(root) / "reference_images" / "images"
        self.contact = np.stack([gray_image(folder / n) for n in CONTACT_REFERENCES])
        self.free = np.stack([gray_image(folder / n) for n in NO_CONTACT_REFERENCES])

    def __call__(self, path):
        x = gray_image(path)
        return bool(np.abs(self.contact - x).mean() < np.abs(self.free - x).mean())


def prepare(root, out, seed=123, group_by="recording", window=100, stride=50, sample_hz=1000., images=True, image_contact_source="annotation"):
    if image_contact_source not in ("annotation", "reference"):
        raise ValueError("Choose annotation or reference image contact filtering")
    root, out = Path(root).resolve(), Path(out).resolve()
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Use a new output directory to preserve the prior split: {out}")
    out.mkdir(parents=True, exist_ok=True)
    records = []
    for joint_file in sorted(root.rglob("joint_states.csv")):
        folder = joint_file.parent
        if not (folder / "contact.csv").exists():
            raise FileNotFoundError(folder / "contact.csv")
        inc, mat = labels_for(folder.relative_to(root))
        records.append(dict(id=folder.relative_to(root).as_posix(), incline=inc, material=mat))
    assignment = split_recordings(records, seed, group_by)
    detector = ReferenceContact(root) if images else None
    image_samples = []
    sum_x, sum_x2, count = np.zeros(9), np.zeros(9), 0
    for i, record in enumerate(records):
        folder = root / record["id"]
        joints = read_csv(folder / "joint_states.csv", ["timestamp"] + CHANNELS)
        contact = read_csv(folder / "contact.csv")
        t, x, y, valid = resample_recording(joints, contact, sample_hz)
        starts = valid_starts(valid, window, stride)
        record.update(split=assignment[record["id"]], cache=f"recording_{i:03d}.npz", windows=len(starts),
                      joint_sha256=sha256(folder / "joint_states.csv"), contact_sha256=sha256(folder / "contact.csv"))
        np.savez_compressed(out / record["cache"], time=t, motor=x, contact=y, starts=starts)
        if record["split"] == "train":
            usable = x[valid].astype(np.float64)
            sum_x += usable.sum(axis=0)
            sum_x2 += (usable ** 2).sum(axis=0)
            count += len(usable)
        if detector:
            audit = dict(images=0, reference_positive=0, annotated_positive=0, comparable=0, disagreements=0)
            with (folder / "image_index.csv").open(newline="") as index_file:
                frames = list(csv.DictReader(index_file))
            for frame in frames:
                p = folder / "images" / Path(frame["filename"]).name
                timestamp = float(frame["timestamp"])
                ci = np.searchsorted(contact[:, 0], timestamp, side="right") - 1
                annotation_valid = 0 <= ci < len(contact) and timestamp <= contact[-1, 0] and timestamp - contact[ci, 0] <= .25
                annotation = bool(contact[ci, 1:].max() > .5) if annotation_valid else False
                reference = detector(p)
                audit["images"] += 1; audit["reference_positive"] += int(reference)
                if annotation_valid:
                    audit["comparable"] += 1; audit["annotated_positive"] += int(annotation)
                    audit["disagreements"] += int(annotation != reference)
                selected = (annotation_valid and annotation) if image_contact_source == "annotation" else reference
                if selected:
                    image_samples.append(dict(path=p.relative_to(root).as_posix(), recording=record["id"],
                                              split=record["split"], incline=record["incline"], material=record["material"]))
            record["image_filter_audit"] = audit
        print(f'{i+1}/{len(records)} {record["id"]}: {len(starts)} windows ({record["split"]})', flush=True)
    if not count or any(sum(r["windows"] for r in records if r["split"] == s) == 0 for s in ("train", "val", "test")):
        raise ValueError("Every split must contain usable windows")
    mean = sum_x / count
    std = np.sqrt(np.maximum(sum_x2 / count - mean ** 2, 1e-8))
    manifest = dict(version=1, root=str(root), seed=seed, group_by=group_by, channels=CHANNELS,
                    inclines=INCLINES, materials=MATERIALS, window=window, stride=stride, sample_hz=sample_hz,
                    image_contact_source=image_contact_source, image_filter_size=224, max_gap_s=.025, label_rule="latest past annotation at window end; max age 0.25s",
                    normalization=dict(mean=mean.tolist(), std=std.tolist(), training_samples=count),
                    records=records, images=image_samples)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--group-by", choices=["recording", "condition"], default="recording")
    p.add_argument("--window", type=int, default=100)
    p.add_argument("--stride", type=int, default=50)
    p.add_argument("--sample-hz", type=float, default=1000.)
    p.add_argument("--no-images", action="store_true")
    p.add_argument("--image-contact-source", choices=["annotation", "reference"], default="annotation")
    args = p.parse_args()
    prepare(args.root, args.out, args.seed, args.group_by, args.window, args.stride, args.sample_hz, not args.no_images, args.image_contact_source)

if __name__ == "__main__":
    main()
