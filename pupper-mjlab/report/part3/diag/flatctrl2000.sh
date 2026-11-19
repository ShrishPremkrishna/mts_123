#!/bin/bash
# Evidence for option (ii) of decision 2026-11-18-w4-p3-candidate (flat control +2000). Evals/export only, no training.
cd ~/wt/w4/pupper-mjlab
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
REF=configs/opt-lab-2/p2/s42/r2-lag
SLOT="uv run --no-sync python scripts/opt_lab_2/gpu_slot_run.py"
RD=~/runs/opt-lab-2/rp-p3-final-flatctrl-kpwide-leg4
C=$RD/model_6492.pt
N=p3-final-flatctrl-kpwide-leg4-it6492
WB=$(grep -o "download_policy.py [^ ]*" $RD.log | tail -1 | cut -d' ' -f2)
$SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval run --checkpoint $C --reference-dir $REF --name $N-seed4321 \
  --conditions flat_dr bumpy_dr --modes suite seam push --seed 4321 --skip-existing
$SLOT -- uv run --no-sync python -m mjlab.scripts.part3_eval ablate --checkpoint $C --reference-dir $REF --name $N \
  --events pd_gains --seed 4321 --skip-existing
$SLOT -- uv run --no-sync python -m mjlab.scripts.part3_export --checkpoint $C --reference-dir $REF --name $N \
  --run-dir $RD ${WB:+--wandb-run $WB}
echo FC_DONE
