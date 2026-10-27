"""Persist text-to-motion samples and explicitly repair a chosen candidate.

Generation needs the separate UniMate environment and GPU. Fitting/repair is CPU
work. Nothing here deploys a policy or labels a kinematic fit physically stable.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

from mjlab.scripts.pupper_foundation import sha256, source_manifest, write_json
from mjlab.tasks.tracking.config.pupper import unimate as U
from mjlab.tasks.tracking.config.pupper.unimate.repair import repair_motion


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  sub = parser.add_subparsers(dest="command", required=True)
  generate = sub.add_parser("generate")
  generate.add_argument("--unimate", type=Path, required=True)
  generate.add_argument("--prompt", action="append", required=True)
  generate.add_argument("--samples", type=int, default=4)
  generate.add_argument("--seed", type=int, default=42)
  generate.add_argument("--cfg", type=float, default=3.0)
  generate.add_argument("--output", type=Path, required=True)
  fit = sub.add_parser("fit")
  fit.add_argument("--raw", type=Path, required=True)
  fit.add_argument("--sample", type=int, required=True)
  fit.add_argument("--name", required=True)
  fit.add_argument("--stride", type=float, default=0.6)
  fit.add_argument("--slow", type=float, default=1.5)
  fit.add_argument("--transition", type=float, default=1.5)
  fit.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  if args.command == "generate":
    path = args.output / "samples.npz"
    if path.exists():
      parser.error("output already contains samples.npz; choose a new directory to preserve the experiment")
    with U.UniMate(args.unimate, log=args.output / "worker.log") as worker:
      raw = worker.generate(args.prompt, n=args.samples, seed=args.seed, cfg=args.cfg)
      np.savez_compressed(path, positions=raw)
      write_json(args.output / "generation.json", {
        "state": "generated_uninspected", "prompts": args.prompt, "seed": args.seed,
        "samples": args.samples, "cfg": args.cfg, "generation_seconds": worker.seconds,
        "raw_sha256": sha256(path), "source": source_manifest(),
      })
    print(f"Saved {len(raw)} candidate motions to {path}; choose a sample after inspection.")
  else:
    if not re.fullmatch(r"[a-z][a-z0-9_]*", args.name):
      parser.error("--name must be lowercase letters, digits and underscores, starting with a letter")
    with np.load(args.raw, allow_pickle=False) as saved:
      raw = saved["positions"]
      if raw.ndim != 4 or not 0 <= args.sample < len(raw):
        parser.error("--sample is outside the generated sample array")
      sample = raw[args.sample].copy()
    final = args.output / f"{args.name}.npz"
    if final.exists():
      parser.error("motion already exists; choose a new output directory or name")
    fitted = U.fit_pupper(sample, stride=args.stride, slow=args.slow)
    fitted.motion.save_npz(args.output / f"{args.name}_before_repair.npz")
    repaired = repair_motion(fitted.motion, transition_s=args.transition)
    repaired.save_npz(final)
    repaired.save_robot_csv(args.output / f"{args.name}_robot.csv")
    report = {
      "state": "kinematic_reference_only", "sample_index": args.sample,
      "raw_file": str(args.raw.resolve()), "raw_sha256": sha256(args.raw),
      "motion_sha256": sha256(final), "fit_options": {"stride": args.stride, "slow": args.slow},
      "before": fitted.stats, "after": repaired.report, "source": source_manifest(),
      "physics_validated": False, "approved_for_hardware": False,
    }
    write_json(final.with_suffix(".json"), report)
    print(json.dumps({"motion": str(final), "before": fitted.stats, "after": repaired.report}, indent=2))
    if not repaired.report["geometry_pass"]:
      raise SystemExit("Geometry checks failed. Inspect the saved candidate and revise the motion before training.")
    print("Geometry checks passed; next inspect physics replay, then train and score the policy.")


if __name__ == "__main__":
  main()
