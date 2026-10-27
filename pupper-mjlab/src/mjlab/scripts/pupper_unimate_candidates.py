"""Fit every saved UniMate sample, preserve failures, and nominate an RL candidate.

This does not train or deploy. A nominal replay with drift can still be a useful
RL candidate: rank it separately from true body tilt, loss of support and effort.
Use --videos on a GPU host with MUJOCO_GL=egl to inspect actual simulated states.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mjlab.scripts.pupper_foundation import sha256, source_manifest, write_json
from mjlab.tasks.tracking.config.pupper import unimate as U
from mjlab.tasks.tracking.config.pupper.unimate.repair import repair_motion


def candidate_assessment(motion, physics: dict) -> dict:
  """A modest standing-dance screening heuristic, explicitly not certification.

  The absolute tilt gate applies only to this standing prompt, not sit/spider or
  arbitrary generated tricks. Position-tracking failure alone is not a rejection.
  Torque/contact are exposed for inspection; neither has a made-up safety limit.
  """
  joint_amplitude = float(np.sqrt(np.mean(np.ptp(motion.joint_pos, axis=0) ** 2)))
  root_amplitude = float(np.max(np.linalg.norm(motion.root_pos - motion.root_pos[0], axis=1)))
  reasons = list(motion.report.get("geometry_issues", []))
  if joint_amplitude < 0.03 and root_amplitude < 0.01:
    reasons.append("sample is effectively static")
  if physics.get("reference_resets", 0):
    reasons.append("physics replay contains a reference reset")
  if physics["peak_body_tilt_from_upright_deg"] > 60:
    reasons.append("standing-dance replay tilts more than 60 degrees; revise before training")
  # A transparent ordering among plausible references, not a pass/fail score.
  rank = (physics["rms_position_error_m"]
          + physics["mean_orientation_error_deg"] / 450
          + physics["fraction_frames_without_foot_contact"] * 0.1)
  return {"candidate_for_rl": not reasons, "reasons": reasons, "ranking_cost": rank,
          "joint_amplitude_rms_rad": joint_amplitude, "root_excursion_m": root_amplitude,
          "ranking_scope": "nominal open-loop replay only; drift is allowed because feedback RL may correct it",
          "trained": False, "approved_for_hardware": False}


def prepare_candidates(raw_file: Path, output: Path, stride: float = 0.4,
                       slow: float = 1.5, transition: float = 1.5,
                       videos: bool = False) -> dict:
  from mjlab.tasks.tracking.config.pupper.play_motion import main as replay

  with np.load(raw_file, allow_pickle=False) as saved:
    raw = saved["positions"]
    if raw.ndim != 4 or raw.shape[1] < 2 or raw.shape[2:] != (19, 3) or not np.isfinite(raw).all():
      raise ValueError("raw positions must be finite (samples, frames>=2, 19, 3)")
  if len(raw) < 1:
    raise ValueError("no generated candidates")
  output.mkdir(parents=True, exist_ok=False)
  report = {"state": "evaluating_candidates", "raw_file": str(raw_file.resolve()),
            "raw_sha256": sha256(raw_file), "source": source_manifest(),
            "fit_options": {"stride": stride, "slow": slow, "transition": transition},
            "candidates": [], "selected_motion": None, "approved_for_hardware": False}
  for index, sample in enumerate(raw):
    directory = output / f"sample_{index:02d}"
    directory.mkdir()
    candidate = {"sample_index": index, "directory": str(directory.resolve()),
                 "candidate_for_rl": False}
    try:
      fitted = U.fit_pupper(sample, stride=stride, slow=slow)
      fitted.motion.save_npz(directory / "before_repair.npz")
      repaired = repair_motion(fitted.motion, transition_s=transition)
      path = repaired.save_npz(directory / "secret_dance.npz")
      repaired.save_robot_csv(directory / "secret_dance_robot.csv")
      candidate.update(motion=str(path.resolve()), motion_sha256=sha256(path),
                       before=fitted.stats, after=repaired.report)
      if not repaired.report["geometry_pass"]:
        candidate["reasons"] = repaired.report["geometry_issues"]
      else:
        replay(path, report_file=directory / "physics.json",
               video_file=directory / "physics.mp4" if videos else None)
        physics = json.loads((directory / "physics.json").read_text())
        candidate.update(candidate_assessment(repaired, physics))
        candidate["physics"] = {key: value for key, value in physics.items() if key != "samples"}
        candidate["video"] = str((directory / "physics.mp4").resolve()) if videos else None
    except Exception as exc:
      # Keep evidence for every failed sample, and let the next independent
      # candidate proceed. Never swallow KeyboardInterrupt/SystemExit.
      candidate.update(candidate_for_rl=False, error=f"{type(exc).__name__}: {exc}")
    write_json(directory / "candidate.json", candidate)
    report["candidates"].append(candidate)
    write_json(output / "candidates.json", report)
  eligible = [candidate for candidate in report["candidates"] if candidate["candidate_for_rl"]]
  if eligible:
    chosen = min(eligible, key=lambda c: c["ranking_cost"])
    report.update(state="candidate_selected_for_inspection", selected_motion=chosen["motion"],
                  selected_sample=chosen["sample_index"], selected_motion_sha256=chosen["motion_sha256"])
  else:
    report["state"] = "no_plausible_candidate_revise_generation_or_repair"
  write_json(output / "candidates.json", report)
  return report


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--raw", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--stride", type=float, default=0.4)
  parser.add_argument("--slow", type=float, default=1.5)
  parser.add_argument("--transition", type=float, default=1.5)
  parser.add_argument("--videos", action="store_true")
  args = parser.parse_args()
  result = prepare_candidates(args.raw, args.output, args.stride, args.slow, args.transition, args.videos)
  print(json.dumps({k: v for k, v in result.items() if k not in ("source", "candidates")}, indent=2))
  if result["selected_motion"] is None:
    raise SystemExit("No sample passed standing-dance screening; inspect candidates.json before revising.")


if __name__ == "__main__":
  main()
