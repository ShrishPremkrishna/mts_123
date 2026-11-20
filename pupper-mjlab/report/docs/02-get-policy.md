# 2 · Get the policy

```bash
cd ~/mts_123/pupper-mjlab/report/docs/scripts
./get_policy.sh bumpy        # -> ~/policies/23pt1499.json
./get_policy.sh flat         # -> ~/policies/13m4vcia.json
./get_policy.sh <entity>/<project>/<run-id>   # any other run
```

The script downloads the run's `policy.json` and runs the course validator. The validator checks the observation layout, network shapes, joint order and gait tables, and prints `Policy looks consistent with the controller config.`
Compare the printed sha256 with the table in [README.md](README.md).

No W&B access? Copy the file from the GPU box. On the GPU box it is at `/home/ubuntu/runs/opt-lab-2/rp-p3-final-{bumpyft,flatctrl}-kpwide-leg4/policy.json`. From the Mac:

```bash
scp -3 prime:/home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/policy.json pi@<pupper>:policies/23pt1499.json
```
