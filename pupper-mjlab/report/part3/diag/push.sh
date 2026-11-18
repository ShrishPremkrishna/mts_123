#!/bin/bash
# DIAGNOSTIC push evidence for the Part 3 candidate decision (flat_dr + bumpy_dr push; part2_eval protocol).
cd ~/wt/w4/pupper-mjlab
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
REF=configs/opt-lab-2/p2/s42/r2-lag
OUT=report/part3/diag/push
SLOT="uv run --no-sync python scripts/opt_lab_2/gpu_slot_run.py"
R=~/runs/opt-lab-2
run() {  # ckpt name seed conds...
  C=$1; N=$2; S=$3; shift 3
  $SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval run --checkpoint $C --reference-dir $REF --name $N \
    --out-root $OUT --conditions "$@" --modes push --seed $S --skip-existing
}
run $R/rp-p3-final-bumpyft-kpwide-leg1/model_4995.pt p3-final-bumpyft-kpwide-leg1-it4995 1234 flat_dr bumpy_dr
run $R/rp-p3-final-bumpyft-kpwide-leg2/model_5494.pt p3-final-bumpyft-kpwide-leg2-it5494 1234 flat_dr bumpy_dr
run $R/rp-p2-s42-r2-cont-1500/model_4496.pt p3-final-P-seed4321 4321 flat_dr
run $R/rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt p3-final-bumpyft-kpwide-leg4-it6492-seed4321 4321 flat_dr
# checkpoint-to-checkpoint push variance for the top candidates (flat_dr, seed 1234)
for li in 4:6400 4:6450 3:5950 4:6050; do leg=${li%%:*}; it=${li##*:}
  run $R/rp-p3-final-bumpyft-kpwide-leg$leg/model_$it.pt p3-final-bumpyft-kpwide-leg$leg-it$it 1234 flat_dr
done
echo PUSH_DONE
