#!/bin/bash
# DIAGNOSTIC (not selection input): seed-4321 flat_dr/bumpy_dr suites + nominal-PD for +500/+1000, both arms.
cd ~/wt/w4/pupper-mjlab
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
REF=configs/opt-lab-2/p2/s42/r2-lag
OUT=report/part3/diag
SLOT="uv run --no-sync python scripts/opt_lab_2/gpu_slot_run.py"
for arm in bumpyft flatctrl; do
  for pt in "1 4995" "2 5494"; do
    set -- $pt; leg=$1; it=$2
    C=~/runs/opt-lab-2/rp-p3-final-$arm-kpwide-leg$leg/model_$it.pt
    N=p3-final-$arm-kpwide-leg$leg-it$it
    $SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval run --checkpoint $C --reference-dir $REF --name $N-seed4321 \
      --out-root $OUT --conditions flat_dr bumpy_dr --modes suite --seed 4321 --skip-existing
    $SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval ablate --checkpoint $C --reference-dir $REF --name $N \
      --out-root $OUT --events pd_gains --seed 4321 --skip-existing
  done
done
echo DIAG_DONE
