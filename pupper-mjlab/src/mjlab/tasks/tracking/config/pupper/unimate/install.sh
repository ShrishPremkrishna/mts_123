#!/usr/bin/env bash
# Install UniMate (text-to-motion) in its own Python 3.10 environment.
#
#   bash install.sh [DIR]      # default DIR: ./UniMate
#
# Clones UniMate at a tested commit into DIR, makes DIR/.venv with uv, and
# downloads the released checkpoint (about 1.2 GB) into DIR/outputs. Safe to
# rerun: finished steps are skipped. UniMate keeps its own environment because
# it pins its own torch and transformers; mjlab talks to it through worker.py.
set -euo pipefail

DIR=${1:-UniMate}
REPO=https://github.com/Friedrich-M/UniMate.git
COMMIT=b78c7807df7cf4231d4e67f8d0ece29c5e11025b
MOTION_COMMIT=ac236251f90e5ca37c444c53ad383fc85de6d833
WEIGHTS_REVISION=174fc755ec248d0c98f27759b7a95648e7f05905
EXP=unimate_uniml3d_f60_v3
STEP=150000

command -v uv >/dev/null || pip install -q uv
if [ ! -d "$DIR/.git" ]; then
  git clone -q "$REPO" "$DIR"
fi
git -C "$DIR" checkout -q "$COMMIT"

UNIMATE_PY="$DIR/.venv/bin/python"
INSTALL_VERSION="$COMMIT-$MOTION_COMMIT-cu128-v3"
environment_healthy() {
  [ -x "$UNIMATE_PY" ] || return 1
  "$UNIMATE_PY" - <<'PY'
import importlib
import sys
import torch
assert sys.version_info[:2] == (3, 10), 'UniMate requires its separate Python 3.10 environment'
assert torch.__version__.split('+')[0] == '2.9.0' and torch.version.cuda == '12.8', 'Incorrect UniMate Torch/CUDA build'
for name in ('huggingface_hub', 'transformers', 'torch_geometric', 'torchdiffeq',
             'spacy', 'sentencepiece', 'Animation', 'Quaternions'):
    importlib.import_module(name)
PY
}
if [ ! -f "$DIR/.venv/.done" ] || [ "$(cat "$DIR/.venv/.done")" != "$INSTALL_VERSION" ] || ! environment_healthy; then
  uv venv -q --allow-existing --python 3.10 "$DIR/.venv"
  uv pip install --python "$UNIMATE_PY" -q "setuptools<81"
  # cu128 wheels run on every current GPU, Blackwell (RTX 5090, RTX PRO 6000) included.
  uv pip install --python "$UNIMATE_PY" -q torch==2.9.0 --torch-backend cu128
  uv pip install --python "$UNIMATE_PY" -q numpy torch-geometric==2.7.0 torchdiffeq==0.2.5 einops==0.8.2 \
    accelerate==1.14.0 tensorboard transformers==5.16.1 sentencepiece==0.2.1 \
    "spacy>=3.8,<3.9" \
    "en-core-web-sm @ https://github.com/explosion/spacy-models/releases/download/en_core_web_sm-3.8.0/en_core_web_sm-3.8.0-py3-none-any.whl" \
    num2words==0.5.14 huggingface_hub==1.29.0 scipy matplotlib imageio imageio-ffmpeg pillow tyro==1.0.12 rich loguru tqdm
  uv pip install --python "$UNIMATE_PY" -q --no-build-isolation "Motion @ git+https://github.com/inbar-2344/Motion.git@$MOTION_COMMIT"
  environment_healthy
  printf '%s\n' "$INSTALL_VERSION" > "$DIR/.venv/.done"
fi
uv pip freeze --python "$UNIMATE_PY" > "$DIR/foundation-environment.txt"

CKPT="$DIR/outputs/$EXP/checkpoints/checkpoint_step_$STEP.pt"
if [ ! -s "$CKPT" ] || [ ! -s "$DIR/outputs/$EXP/config.json" ] || [ ! -s "$DIR/outputs/$EXP/dataset_stats.npy" ]; then
  # Use the library API: hf's console entry point is not installed by every hub
  # distribution, and the model worker needs the library anyway. Do not swallow
  # a download exception or label a partial checkout ready.
  "$UNIMATE_PY" - "$DIR/outputs" "$EXP" "$STEP" "$WEIGHTS_REVISION" <<'PY'
from pathlib import Path
import sys
from huggingface_hub import snapshot_download
root, experiment, step, revision = sys.argv[1:]
snapshot_download(repo_id='Linzhan/UniMate', revision=revision, local_dir=root,
    allow_patterns=[f'{experiment}/*.json', f'{experiment}/*.npy',
                    f'{experiment}/checkpoints/checkpoint_step_{step}.pt'], max_workers=2)
for relative in ('config.json', 'dataset_stats.npy', f'checkpoints/checkpoint_step_{step}.pt'):
    path = Path(root) / experiment / relative
    if not path.is_file() or path.stat().st_size == 0:
        raise SystemExit(f'Incomplete UniMate download: {path}')
PY
fi
echo "UniMate ready: $DIR ($EXP, step $STEP)"
