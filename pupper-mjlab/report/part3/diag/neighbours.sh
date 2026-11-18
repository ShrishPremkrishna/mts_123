#!/bin/bash
# DIAGNOSTIC (not selection input): checkpoint-to-checkpoint variance. flat_dr suite + nominal-PD, eval seed 1234,
# on the neighbours of a checkpoint. Usage: neighbours.sh <arm> <leg>:<iter> [<leg>:<iter> ...]
cd ~/wt/w4/pupper-mjlab
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
REF=configs/opt-lab-2/p2/s42/r2-lag
OUT=report/part3/diag/neighbours
SLOT="uv run --no-sync python scripts/opt_lab_2/gpu_slot_run.py"
arm=$1; shift
for li in "$@"; do
  leg=${li%%:*}; it=${li##*:}
  C=~/runs/opt-lab-2/rp-p3-final-$arm-kpwide-leg$leg/model_$it.pt
  N=p3-final-$arm-kpwide-leg$leg-it$it
  [ -f "$C" ] || { echo "missing $C"; continue; }
  $SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval run --checkpoint $C --reference-dir $REF --name $N \
    --out-root $OUT --conditions flat_dr --modes suite --skip-existing
  $SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval ablate --checkpoint $C --reference-dir $REF --name $N \
    --out-root $OUT --events pd_gains --skip-existing
done
echo NEIGH_DONE
