---
date: 2026-10-27T15:16
tags: [cs123, foundation]
---
# Implicit compliance

This implements Optional Lab 1 as a reproducible experiment and an opt-in streaming capability. It does **not** establish general terrain understanding or stable walking from a fixed right-front-leg dataset. Source: [course handout](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-1.html), retrieved October 27, 2026. The supplied notebooks are preserved in the course library; this package replaces their incomplete execution path.

## Data contract and evaluation

- Nine named channels are three positions, three velocities, and three **efforts**. Effort is not acceleration. The export carries the exact order, normalizer, physical sample rate and window length.
- Split entire recordings before creating windows or selecting images. `--group-by condition` holds out complete incline/material combinations as a harder second experiment. This is not a claim of independent collection sessions: the archive does not provide reliable session IDs for all files.
- Sample uniformly at 1,000 Hz using timestamps, reject sensor gaps over 25 ms, and use only observed contact annotation intervals with latest-past labels at the window's end. This intentionally avoids the course notebook's future interpolation/extrapolation and random-window leakage. Default history: 100 samples (99 ms between first and last sample), stride 50.
- Normalize using **training recordings only**. Incline/material losses and metrics use contact samples only. Missing classes in held-out splits are reported as `null` recall with zero support, not perfect accuracy.
- The explicit model is a two-head ResNet-50 initialized from ImageNet. The implicit model is a small temporal CNN with contact/incline/material heads. PCGrad projects conflicting task gradients; `--no-pcgrad` is a controlled ablation. The explicit baseline fully fine-tunes the backbone; it does not use the original notebook's partially specified LoRA wrapper.
- Select the checkpoint only with validation metrics. Run the untouched test set once after model selection. Compare per-class recall, contact false positives/sensitivity and held-out support, not just aggregate accuracy.

The supplied image reference filter disagrees substantially with the recorded detector values on the linked ZIP: the course's expected retained-image count is not a reproducible guarantee. Preparation records, for every recording, reference-filter positives, timestamp-annotation positives and disagreements. `--image-contact-source annotation` (default) uses the existing `contact.csv` aligned through `image_index.csv` for the explicit training set. `reference` runs the course's MAD-filter ablation. The handout says contact detection during collection also used image references at 10 Hz. These are alternative noisy labels, not manually verified ground truth. Both pipelines use the same whole-recording split.

## Reproduce

Run from the outer `cs123` repository, in an environment with `implicit_compliance/requirements.txt` installed. Keep datasets and model weights under ignored `artifacts/`.

```sh
python -m implicit_compliance.data artifacts/tactile_data artifacts/compliance/prepared --group-by recording --image-contact-source annotation
python -m implicit_compliance.train train --manifest artifacts/compliance/prepared/manifest.json --out artifacts/compliance/implicit --kind implicit --epochs 50 --batch-size 256
python -m implicit_compliance.train train --manifest artifacts/compliance/prepared/manifest.json --out artifacts/compliance/explicit --kind explicit --epochs 10 --batch-size 16
python -m implicit_compliance.train evaluate --manifest artifacts/compliance/prepared/manifest.json --checkpoint artifacts/compliance/implicit/best.pt --out artifacts/compliance/implicit/test.json
python -m implicit_compliance.train evaluate --manifest artifacts/compliance/prepared/manifest.json --checkpoint artifacts/compliance/explicit/best.pt --out artifacts/compliance/explicit/test.json
python -m implicit_compliance.train export --checkpoint artifacts/compliance/implicit/best.pt --out artifacts/compliance/deploy
python -m implicit_compliance.audit artifacts/compliance/prepared/manifest.json artifacts/compliance/split-audit.json
python -m pytest implicit_compliance/tests -q
```

The fixed diagnostic below freezes ImageNet features, standardizes them using training images only, fits balanced L2 logistic heads with `C=1`, and evaluates validation only. It never constructs a test dataset. Use it to test the effect of full-network finetuning; it does not repair weak labels or establish new test performance.

```sh
python -m implicit_compliance.ablation artifacts/compliance/prepared/manifest.json artifacts/compliance/frozen-linear-validation
```

`data.extract_dataset(zip_path, destination)` extracts the supplied ZIP after checking traversal and symlink entries. Do not copy credentials into notebooks or configuration. For offline explicit training, cache the official PyTorch ResNet-50 weights first (`TORCH_HOME` controls that directory). `--max-samples` is only an infrastructure smoke test; every output made with it is marked limited.

The original source `.ipynb` files contain invalid placeholders, lookup paths that discard `data_dir`, and a training TODO followed by an already-implemented update. Those mistakes are not propagated here. This package performs one optimizer update per batch.

## Contact-triggered trajectory pause

`ContactPredictor(export_dir).push(timestamp, nine_channel_vector)` returns a prediction at a bounded inference frequency, or `None` when evidence is not ready. `motor_channels(...)` maps ROS JointState names explicitly and rejects missing effort. Never substitute commanded torque for measured effort without a separate data-transfer study.

Pass each **new** probability to `ContactPauseClock.advance(monotonic_time, probability)`. Pass `None` on intervening control ticks. Its returned trajectory time replaces wall time in the triangular-trajectory phase calculation; it freezes that time for one second after two high-confidence readings. It requires three low-contact readings and a cooldown to rearm. No callbacks sleep and no stale inference is treated as new evidence. A time rewind invalidates the event state. This class has no direct motor or ROS side effects, so the control owner can integrate it without two motor publishers competing.

Before enabling it on the robot: record a shadow-mode stream, verify named channels/unit scaling and actual timestamp rate, benchmark inference latency/CPU load alongside the normal stack, and measure false triggers and delayed/missed contacts on the intended trajectory. The model may need robot-specific data. Its terrain heads must not control locomotion merely because they return a class.

## Outputs

A prepared dataset has a frozen split manifest and one compressed numerical cache per recording. Each run stores all arguments, manifest checksum, train/validation counts, dependency version, seed, validation history, and best/last checkpoints. Export checks numerical parity and writes `contact.pt` plus a checksum-protected `metadata.json`. Test reports explicitly say **course dataset, not hardware validation**. Actual run results belong in `results/`; do not replace measurements with the course notebook's quoted accuracies.
