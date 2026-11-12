---
date: 2026-10-27T15:34
tags: [cs123, foundation]
---
# Pupper releases and evidence

This folder contains release tools, local validation, recorded experiments and the Lab 6 submission handoff. The [repository map](../README.md) describes the code. The [project brief](../final_project/README.md) holds the current project discussion.

## Checkpoint and operation

Run `python3 foundation/scripts/session_status.py` from the repository root. It reads the [checkpoint](results/interruption-recovery.json) and local references. `--verify-inputs` also requires the saved local inputs and verifies their pinned digests. It performs no robot connection.

The checkpoint states when its facts were observed. Its latest owned physical session is R5 from November 3; R6 was staged on November 4. Current Pi processes, ownership and loaded code require a live check when authorized. Release hashes, paths and held candidates are in that record. The [robot procedure](HARDWARE_CHECKLIST.md) explains the existing commands and their effects.

## Evidence by question

| Question | Starting record |
| --- | --- |
| What has been observed on Pupper, and what remains for Lab 6? | [November 3 sprint](results/harness-sprint-20261103.json), [writer packet](submission/lab6/README.md), [engineering notes](WORKSTREAMS.md) |
| What software was tested for R6? | [Checkpoint release selection](results/interruption-recovery.json), including archive and manifest hashes |
| Where are earlier IMU failures and parser investigations? | [Failure record](results/workstream1-hardware-imu-failure-20261101.json), [parser repair](results/imu-parser-candidate-20261101.json), [camera comparison](results/workstream1-camera-sensor-consistency-20261102.json) |
| What motion constraints and reference corrections were established? | [Driver limits](results/local-driver-position-limit-audit.json), [encoder semantics](results/encoder-hardware-semantics-review.json), [absolute reference correction](results/optional2-class-seed-correction.json) |
| Where are the selected Spider and Optional Lab 2 findings? | [Spider study](results/spider-encoder-native-reviewed.json), [Bumpy comparison](results/optional2-checked-bumpy-review.json), [Mystery trials](results/optional2-mystery-reviewed.json) |
| Where are generated motion and secret-gate results? | [Optional Lab 3 context](results/optional3-context-20261101.json), [motion/contact evaluation](results/generated-bob-contact-extent-20261101.json) |
| Where is the broader historical capability narrative? | [November 4 foundation overview](https://github.com/ShrishPremkrishna/mts_123/blob/59db01658ac9fe83e86ccc0b52574e5a28f43de3/foundation/README.md) |

Reports under [results](results/) preserve their original dates, inputs and conclusions. Their recommendations describe those experiments. API acceptance, simulation outcomes and observed physical motion are distinct evidence. Runtime selection and present project priorities belong in the checkpoint and project brief.

## Local numerical setup

The Mac runs CPU simulation and numerical tests in a checkout-local `.venv`.
For a fresh checkout, run this from its root in zsh or bash with `python3.12` installed. It installs `uv` locally, uses the checked-in training lock and adds the Mac lab-report and compliance packages.

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install uv==0.13.6
source .venv/bin/activate
UV_PROJECT_ENVIRONMENT="$PWD/.venv" uv sync --project pupper-mjlab --locked --inexact --extra cpu --python 3.12
uv pip install --python .venv/bin/python -r foundation/configs/macos-numerical-extras.txt \
  -c <(uv export --project pupper-mjlab --locked --extra cpu --no-emit-project --no-hashes)
uv pip check --python .venv/bin/python
```

The regression runner checks that `mjlab` loads from this checkout's `pupper-mjlab/src`. GPU training uses a CUDA host. API/web development has its [own setup](../gemini_pupper/README.md#offline-checks).

```sh
python3 foundation/scripts/check_foundation.py --output /private/tmp/pupper-validation-NEW
python3 foundation/scripts/check_ros_offline.py --output /private/tmp/pupper-ros-NEW
```

Both commands run broad offline checks; the [root quick reference](../README.md#choose-the-check-for-the-task) lists focused checks. Replace `NEW` with a new output name for each run. Explicit output paths avoid writing through the historical artifacts alias, and existing output directories are refused.

The first discovers the API and numerical environments, with explicit `--api-python` and `--numerical-python` overrides. The second requires Docker Desktop and the image described in [ros-offline.Dockerfile](scripts/ros-offline.Dockerfile). On this Mac, start the engine with `open -a Docker` when needed; `docker info` verifies readiness. Inspect the cached image with `docker image inspect cs123-ros-jazzy-slew-check:latest` before deciding a build is needed. The runner uses networking-disabled containers without device mounts. [Recorded ROS validation](results/workstream1-offline-ros-20261102.json)

## Archived inputs

The local `artifacts/` alias points into the older checkout. Additional verified archives live under `/Users/shrish/workspace/pupper-foundation-artifacts/`. These are machine-local assets. Dated reports identify the relevant archive, input identities and checksums. Training `/content` and temporary directories are transient storage.

Existing long-run tools cover [checkpoint collection](scripts/preserve_colab_run.py), [verified restore](scripts/restore_colab_snapshot.py) and [bounded final downloads](scripts/download_colab_archive.py). A completed run needs its final exit record and complete archive; an intermediate snapshot describes only its capture time.
