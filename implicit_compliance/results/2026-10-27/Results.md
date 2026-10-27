---
date: 2026-10-27T15:31
tags: [cs123, experiment]
---
# Optional Lab 1: measured baseline

The complete local implicit run finished 50 epochs on the supplied course dataset. Model selection used validation only; epoch 49 had the highest mean validation balanced accuracy across the three heads. Training used 31,033 windows, validation 5,394, and the untouched test split 7,269. Each complete acquisition folder belongs to exactly one split. No sample cap was used. This took 915.5 seconds on the development CPU.

| Held-out result | Accuracy | Balanced accuracy |
| --- | ---: | ---: |
| Contact, threshold 0.5 | 83.75% | 85.59% |
| Incline, contact windows only | 44.26% | 41.83% |
| Material, contact windows only | 9.50% | 8.13% |

At the streaming entry threshold of 0.8, contact precision is 98.64%, sensitivity 70.09%, and false-positive rate 1.35%. Those are **window-level dataset statistics**, not robot contact-event rates. The conservative threshold misses about 30% of annotated contact windows; hysteresis and a one-second pause do not fix that underlying limitation. The terrain heads are unsuitable for controlling locomotion based on these measurements.

This result does not reproduce the notebook's quoted roughly 95%/90%/80% figures. Our split keeps whole recordings apart, uses true timestamps and motor effort, and avoids interpolation outside observed annotations. The source notebook's randomly split overlapping windows answer an easier question. Neither result establishes performance on arbitrary real-world surfaces or another robot.

The recorded source image contact filter also needs care: among 6,621 indexed experiment images, the 224-pixel MAD filter retains 278, while timestamp-aligned recorded contact values retain 3,315. On 6,610 comparable images they disagree 3,037 times. The course handout's Data Collection section says the recorded contact detector itself used reference-image differences at 10 Hz. These values are therefore detector labels, not human ground truth. The saved ZIP contains additional reference images and other images beyond the indexed experiment frames. The disagreements have not been manually adjudicated. The explicit baseline uses the recorded values by default, while retaining the MAD filter as a separately selectable ablation.

The full explicit ResNet-50 baseline completed ten epochs on the Colab GPU in 64.7 seconds, using 2,289 training images and 387 validation images. It used PCGrad/AdamW, learning rate 0.001, batch size 64, seed 123, and ImageNet initialization. Epoch 9 won the prespecified validation selection rule. Its unchanged test evaluation on 639 images produced:

| Explicit image classifier | Validation accuracy at selected epoch | Held-out accuracy | Held-out balanced accuracy |
| --- | ---: | ---: | ---: |
| Incline | 79.59% | 54.62% | 54.62% |
| Material | 35.14% | 15.34% | 13.97% |

These are different input modalities and sample units from the motor model, so this is not a paired claim that one is better. Neither material head is ready to influence behavior. The explicit GPU source revision is `16068a2559fed3035de85b6d8493c59634c5cb65`; its exact Python source hashes and CUDA configuration remain unchanged in `explicit-gpu-run.json`. `explicit-gpu-history.json` preserves every epoch and `explicit-gpu-test.json` identifies the evaluated checkpoint and manifest by SHA-256. This GPU result supersedes the interrupted duplicate CPU explicit run, not the completed CPU implicit run.

The split has 40 training acquisitions, eight validation acquisitions and eight test acquisitions. Training contains all six incline and all six material classes. Validation contains no 0° or 75° images and no foam; test contains no 30° images and no `none` or `slip` material images. Balanced accuracy averages only classes with support, and missing-class recall remains null. Four validation acquisitions and five test acquisitions use incline/material combinations absent from training. Consequently these numbers combine acquisition shift and some new condition combinations, and cannot establish coverage of all six classes. The small number of independent recordings matters more than the number of correlated frames.

The reproducible audit in `split-coverage-and-alignment.json` found no byte-identical selected images crossing splits. On training and validation frames, median label age is about 49 and 43 ms, respectively; median contact sampling is 100 ms and image sampling 333 ms. A descriptive ±100 ms alignment perturbation changes approximately 3–5% of valid contact labels; no timing offset was selected or applied. This is far smaller than the 46% disagreement with the reference filter, so modest timestamp offsets alone are unlikely to explain that disagreement. Reference frames are 640×480 and experiment frames 320×240; both are square-cropped and resized, but sensor appearance/reference calibration remains a hypothesis requiring manual inspection, not a diagnosed correction.

The one fixed diagnostic froze the ImageNet backbone and fitted two balanced, regularized linear heads on training recordings. It evaluated only the original 387 validation images and never constructed a test dataset. In 293.6 seconds on the development CPU, it reached 78.29% incline accuracy (81.70% balanced) and 20.93% material accuracy (18.57% balanced). The full-finetuning baseline's selected validation balanced accuracies were 83.27% and 32.56%, respectively. Freezing the backbone did not improve either measured validation head; this single experiment does not establish the cause of the generalization gap. Its fixed configuration, source hashes and confusion matrices are in `explicit-frozen-linear-validation.json`. No extra test evaluation was run and no terrain head was promoted.

Neither model choice resolves weak contact labels or absent validation classes. Any future model chosen after observing the present test should be assessed on fresh held-out recordings, including every intended surface and incline, before deployment claims. The local and GPU manifests have been compared field by field: only their absolute dataset-root paths differ; split memberships, file hashes, normalizer, labels and selected images are identical. Both full-file hashes and the root-independent semantic hash are recorded in the split audit.

The exported motor model is 133,593 bytes. TorchScript/eager numerical parity passed. On this development CPU with one thread and batch 1, 100 timed inferences after warmup gave median 0.101 ms and 95th percentile 0.105 ms. This is not a Raspberry Pi performance measurement. The export remains experimental until a shadow-mode robot stream verifies channel scaling, calibration, latency, missed contacts and false triggers on the intended trajectory.

Machine-readable evidence is adjacent: `implicit-run.json`, `implicit-history.json`, `implicit-test.json`, `implicit-export-metadata.json`, `implicit-export-benchmark.json`, and `dataset-split-and-audit.json`. Full numerical caches and trained weights are in ignored `artifacts/implicit_compliance/`. The first local run began from the uncommitted implementation; the recorded source revision alone does not describe that dirty tree. Training/model operators were unchanged while reporting/export code developed. Subsequent committed GPU reruns snapshot exact source files and hashes and provide the canonical reproducible source provenance.
