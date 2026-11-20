import json, sys
sys.argv = [sys.argv[0]]
exec(open("loop_json.py").read().split("pols = {")[0])
pols = {n: json.load(open(p)) for n, p in [("p3", "/home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/policy.json"), ("fasx", "fasx/policy.json")]}
for name, pol in pols.items():
    for I in (0.002, 0.003, 0.004):
        for nz in ((0.0, 0.0), (0.002, 0.01)):
            r = run(pol, I, 16, noise=nz)
            print(name, I, "noise", nz, {k: round(v, 4) for k, v in r.items()}, flush=True)
