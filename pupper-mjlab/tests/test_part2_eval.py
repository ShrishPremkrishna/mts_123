"""CPU-only tests for the Opt Lab 2 extended evaluation harness (no GPU rollout)."""

import json
import math
from typing import Any

import numpy as np
import pytest

from mjlab.scripts import part2_eval as pe

DT = 0.02
STEPS = 1000
RANGES = [(-1.5, 1.5), (-0.5, 0.5), (-2.0, 2.0)]


def _schedule(name):
  segs = dict(pe.SCHEDULES)[name]
  return pe.build_schedule(segs, DT, STEPS)


def test_schedules_shape_ranges_and_hash():
  cmd = pe.build_all_schedules(DT, STEPS)
  assert cmd.shape == (16, STEPS, 3)
  assert len({n for n, _ in pe.SCHEDULES}) == 16
  pe.check_ranges(cmd, RANGES)
  assert np.abs(cmd[..., 0]).max() == pytest.approx(1.5)
  # Every schedule starts standing (settle from spawn).
  assert np.all(cmd[:, :100] == 0)
  assert pe.schedule_hash(pe.SCHEDULES, DT, STEPS) == pe.schedule_hash(
    pe.SCHEDULES, DT, STEPS
  )
  assert pe.schedule_hash(pe.SCHEDULES, DT, STEPS) != pe.schedule_hash(
    pe.SCHEDULES, DT, 999
  )
  with pytest.raises(ValueError):
    pe.check_ranges(cmd * 2, RANGES)
  with pytest.raises(ValueError):
    pe.build_schedule((("hold", 30.0, (0, 0, 0)),), DT, STEPS)


def test_ramp_reaches_target_and_pads_with_stand():
  cmd = pe.build_schedule((("ramp", 1.0, (0, 0, 0), (1.5, 0, 0)),), DT, 100)
  assert cmd[49, 0] == pytest.approx(1.5)
  assert cmd[0, 0] == pytest.approx(1.5 / 50)
  assert np.all(cmd[50:] == 0)


def test_labels_reversal_stop_turn_steady():
  rev = pe.label_schedule(_schedule("reversal_fwd_first"), DT)
  # stand 2 s, ramp 1 s, +1.5 4 s -> flip to -1.5 at 7 s (step 350), back at 11 s.
  assert np.flatnonzero(rev["reversal_onset"]).tolist() == [350, 550]
  assert np.all(rev["context"][350:450] == pe.REVERSAL)
  assert rev["context"][450] == pe.STEADY
  assert rev["context"][349] == pe.STEADY
  assert rev["since_change"][449] == pytest.approx(1.98)
  # Ramps change every step.
  assert rev["since_change"][120] == 0.0

  stops = pe.label_schedule(_schedule("stops_fwd"), DT)
  onsets = np.flatnonzero(stops["stop_onset"])
  assert len(onsets) == 3 and onsets[0] == 250  # 2 + 1 + 2 s
  assert np.all(stops["context"][250:350] == pe.STOP)
  assert stops["context"][350] == pe.STEADY
  assert not stops["reversal_onset"].any()

  turn = pe.label_schedule(_schedule("turn_in_place"), DT)
  assert turn["context"][150] == pe.TURN and turn["context"][50] == pe.STEADY
  side = pe.label_schedule(_schedule("sidestep"), DT)
  assert side["context"][150] == pe.TURN

  mixed = _schedule("mixed")
  lab = pe.label_schedule(mixed, DT)
  # +0.5 -> -1.0 flips sign but |0.5| < 1: not a reversal; -1.0 -> +1.0 is.
  assert lab["reversal_onset"].sum() == 1 and lab["stop_onset"].sum() == 1
  k = int(np.flatnonzero(lab["reversal_onset"])[0])
  assert mixed[k - 1, 0] == -1.0 and mixed[k, 0] == 1.0

  ramp = pe.label_schedule(_schedule("ramp_fwd"), DT)
  assert np.all(ramp["context"] == pe.STEADY)
  seam = pe.label_schedule(_schedule("gait_seam"), DT)
  assert not seam["reversal_onset"].any() and not seam["stop_onset"].any()
  moderate = pe.label_schedule(_schedule("reversal_moderate"), DT)
  assert moderate["reversal_onset"].sum() == 3


def _synthetic(T=600, N=4, dt=DT) -> dict[str, Any]:
  # Schedule 0: stand; schedule 1: +1.5 from t=0 then reversal to -1.5 at 6 s.
  s0 = np.zeros((T, 3))
  s1 = np.zeros((T, 3))
  s1[:, 0] = 1.5
  s1[300:, 0] = -1.5
  env_schedule = np.array([0, 0, 1, 1])
  mass = np.array([3.0, 3.5, 3.0, 4.0])
  g = 9.81
  cmd = np.stack([s0, s1])[env_schedule].transpose(1, 0, 2)
  velocity = cmd.copy()
  velocity[..., 0] -= np.sign(cmd[..., 0]) * 0.5  # 0.5 m/s short of the command
  contact = np.ones((T, N, 4), bool)
  force = np.broadcast_to((mass * g / 4)[None, :, None], (T, N, 4)).copy()
  force[100, 2, 0] = 2.0 * mass[2] * g  # a 2 BW spike on env 2
  heading = np.zeros((T, N))
  heading[:, 2] = np.deg2rad(1.0) / 10.0 * np.arange(T) * dt  # 1 deg / 10 s
  terminated = np.zeros((T, N), bool)
  terminated[320, 3] = True  # during the reversal window
  terminated[50, 0] = True  # standing env, steady
  hind = np.zeros((T, N, 2))
  hind[..., 0] = 0.1  # right outward
  hind[..., 1] = -0.1  # left outward
  torque = np.zeros((T, N, 12))
  torque[:, 2, 0] = 3.0
  return dict(
    schedule_cmd=np.stack([s0, s1]),
    env_schedule=env_schedule,
    velocity=velocity,
    joint_vel=np.full((T, N, 12), 0.2),
    torque=torque,
    foot_force=force,
    foot_contact=contact,
    foot_speed=np.full((T, N, 4), 0.1),
    heading=heading,
    hind_delta=hind,
    terminated=terminated,
    timeout=np.zeros((T, N), bool),
    mass_kg=mass,
    dt=dt,
    gravity=g,
    effort_limit=3.0,
    schedule_names=["stand", "rev"],
  )


def test_metrics_on_synthetic_arrays():
  m = pe.compute_suite_metrics(**_synthetic())
  assert m["falls_total"] == 2
  assert m["falls_by_category"] == dict(steady=1, turn=0, stop=0, reversal=1)
  assert m["falls_by_schedule"] == dict(stand=1, rev=1)
  assert m["category_robot_events"]["reversal"] == 2
  assert m["category_robot_runs"]["reversal"] == 2
  assert m["category_robot_seconds"]["reversal"] == pytest.approx(2 * 100 * DT)
  assert m["track_err_fwd"] == pytest.approx(0.5)
  assert m["track_err_back"] == pytest.approx(0.5)
  assert m["mean_vx_fwd"] == pytest.approx(1.0)
  assert m["mean_vx_back"] == pytest.approx(-1.0)
  # Body-weight normalisation uses each env's own mass.
  assert m["foot_force_stand_mean_bw_per_foot"] == pytest.approx(0.25)
  assert m["foot_force_stand_total_bw"] == pytest.approx(1.0)
  assert m["foot_force_peak_bw"] == pytest.approx(2.0)
  assert m["foot_force_p99_bw"] == pytest.approx(0.25)
  # Drift: env 2 turns 1 deg per 10 s; env 3 holds heading.
  d = m["heading_drift_deg_per_10s_fwd"]
  assert d["max"] == pytest.approx(1.0)
  assert d["mean"] == pytest.approx(0.5)
  assert m["hind_splay_rad_fwd"] == pytest.approx(0.1)
  assert m["stand_jitter_rad_s"] == pytest.approx(0.2)
  assert m["slip_m_s"] == pytest.approx(0.1)
  assert m["top_speed_fwd"] == pytest.approx(1.0)
  assert m["top_speed_back"] == pytest.approx(-1.0)
  # One of 12 joints of one of 4 envs saturates.
  assert m["torque_sat_frac"] == pytest.approx(1 / 48, rel=1e-2)
  assert m["torque_abs_max_nm"] == pytest.approx(3.0)
  # Per-run max median: env 2 has a 2 BW spike, the others 0.25 -> median 0.25.
  assert m["foot_force_runmax_median_bw"] == pytest.approx(0.25)
  assert m["foot_force_runmax_robot_runs"] == 4
  assert m["foot_force_p999_bw"] == pytest.approx(0.25)
  # Per-fall detail: env 0 (stand) at step 50, 1 s into its first episode.
  d = {(f["env"], f["time_s"]): f for f in m["falls_detail"]}
  assert d[(0, 1.0)]["episode_age_s"] == pytest.approx(1.0)
  assert d[(0, 1.0)]["since_command_change_s"] == pytest.approx(1.0)
  assert d[(3, 6.4)]["category"] == "reversal"
  assert d[(3, 6.4)]["since_command_change_s"] == pytest.approx(0.4)
  assert m["falls_soon_after_reset"] == 1
  assert m["falls_soon_after_reset_by_category"]["steady"] == 1


def test_steady_window_excludes_post_reset_and_terminal_samples():
  data = _synthetic()
  m = pe.compute_suite_metrics(**data)
  # Env 0 falls at step 50: the terminal sample and 2 s after are not steady.
  # stand env 0: steady samples 100..599 minus {50} minus 51..150 (age<2 s).
  stand_env0 = 599 - 151 + 1  # steps 151..599 for env 0
  assert m["steady_samples"]["stand"] == stand_env0 + 500


def test_top_speed_excludes_reset_windows():
  T, N = 200, 1
  vx = np.ones((T, N))
  vx[100] = 50.0  # a glitch sample right after a reset
  done = np.zeros((T, N), bool)
  done[99] = True
  alive = ~done
  top = pe.top_speed(vx, alive, done, DT)
  # The only windows containing sample 100 start at >= 100 (after the reset).
  assert top["fwd"] == pytest.approx((50 + 49) / 50)
  done2 = done.copy()
  done2[110] = True
  top2 = pe.top_speed(vx, ~done2, done2, DT)
  assert top2["fwd"] == pytest.approx(1.0)


def test_heading_drift_units_and_unwrap():
  T = 251
  h = np.deg2rad(2.0) / 10.0 * np.arange(T) * DT + math.pi - 0.01
  h = (h + math.pi) % (2 * math.pi) - math.pi  # wraps across +-pi
  d = pe.heading_drift(h[:, None], np.ones((T, 1), bool), DT)
  assert d["windows"] == 1 and d["mean"] == pytest.approx(2.0)
  assert d["signed_mean"] == pytest.approx(2.0)
  neg = pe.heading_drift(-h[:, None], np.ones((T, 1), bool), DT)
  assert neg["mean"] == pytest.approx(2.0) and neg["signed_mean"] == pytest.approx(-2.0)
  # Windows shorter than 1 s are ignored.
  short = pe.heading_drift(h[:40, None], np.ones((40, 1), bool), DT)
  assert short["windows"] == 0 and short["mean"] is None


def test_hind_splay_sign_matches_fk():
  mujoco = pytest.importorskip("mujoco")
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import (
    DEFAULT_POSE,
    FOOT_SITE_NAMES,
    JOINT_NAMES,
    PUPPER_XML,
  )

  model = mujoco.MjModel.from_xml_path(str(PUPPER_XML))
  data = mujoco.MjData(model)

  def foot_y(q, site):
    data.qpos[:] = 0
    data.qpos[3] = 1
    for name, v in zip(JOINT_NAMES, q, strict=True):
      data.qpos[model.jnt_qposadr[model.joint(name).id]] = v
    mujoco.mj_kinematics(model, data)
    return float(data.site(site).xpos[1])

  sites = dict(leg_back_r_2=FOOT_SITE_NAMES[2], leg_back_l_2=FOOT_SITE_NAMES[3])
  for joint, sign in zip(pe.HIND_ABDUCTION_JOINTS, pe.HIND_OUTWARD_SIGN, strict=True):
    site = sites[joint]
    y0 = foot_y(DEFAULT_POSE, site)
    q = list(DEFAULT_POSE)
    q[JOINT_NAMES.index(joint)] += 0.05 * sign  # "outward" per the encoded sign
    y1 = foot_y(q, site)
    assert abs(y1) > abs(y0) + 0.003, (joint, y0, y1)
  assert pe.outward_splay(np.array([0.1, -0.1])) == pytest.approx(0.1)


def test_push_schedule_and_metrics():
  T, N = 1500, 8
  step, mag, sign = pe.make_push_schedule(N, DT, T, seed=1)
  valid = step >= 0
  assert valid.sum(axis=1).min() >= 8
  assert set(np.unique(mag[valid])) <= {0.5, 0.65, 0.8}
  assert set(np.unique(sign[valid])) <= {-1.0, 1.0}
  for e in range(N):
    s = step[e][step[e] >= 0]
    assert s.min() * DT >= pe.PUSH_FIRST_S
    assert np.diff(s).min() * DT >= pe.PUSH_PERIOD_S - 2 * pe.PUSH_JITTER_S - 1e-9
    assert s.max() + 100 <= T
  same = pe.make_push_schedule(N, DT, T, seed=1)
  assert np.array_equal(same[0], step)

  # Hand-built: env 0 (stand) falls 1 s after its first push; envs 1-3 move.
  push_step = np.array([[200, 300, 700], [200, 400, -1], [200, -1, -1], [200, -1, -1]])
  push_mag = np.array([[0.8, 0.5, 0.65], [0.5, 0.8, 0.0], [0.5, 0, 0], [0.5, 0, 0]])
  terminated = np.zeros((1000, 4), bool)
  terminated[250, 0] = True
  terminated[230, 3] = True  # run_-1.5 falls 0.6 s after a 0.5 m/s push
  vx = np.zeros((1000, 4))
  vx[:, 1:] = [0.4, 1.2, -0.3]
  vx[210:300, 1] = 9.0  # inside a push window: excluded from achieved vx
  m = pe.compute_push_metrics(
    push_step=push_step,
    push_mag=push_mag,
    env_condition=np.array([0, 1, 2, 3]),
    terminated=terminated,
    timeout=np.zeros_like(terminated),
    dt=DT,
    vx=vx,
  )
  # Env 0's push at 300 is within 2 s of the reset (sample 251) -> not counted.
  assert m["overall"] == dict(pushes=6, falls=2, rate=pytest.approx(1 / 3))
  assert m["standing"] == dict(pushes=2, falls=1, rate=0.5)
  assert m["by_condition"]["stand"]["0.8"] == dict(pushes=1, falls=1, rate=1.0)
  assert m["by_condition"]["stand"]["0.65"] == dict(pushes=1, falls=0, rate=0.0)
  assert m["by_magnitude"]["0.5"] == dict(pushes=3, falls=1, rate=pytest.approx(1 / 3))
  assert m["falls_outside_push_windows"] == 0
  assert m["by_condition"]["walk_+0.5"]["achieved_vx_m_s"] == pytest.approx(0.4)
  assert m["by_condition"]["run_+1.5"]["achieved_vx_m_s"] == pytest.approx(1.2)
  # Ranked: moving at 0.5 m/s pooled over the moving conditions; standing at the rung.
  r = pe.rank_push(m, standing_rung=0.8)
  assert r["moving_at_rung"] == dict(pushes=3, falls=1, rate=pytest.approx(1 / 3))
  assert r["standing_at_rung"] == dict(pushes=1, falls=1, rate=1.0)
  assert r["moving_at_rung_by_condition"]["run_-1.5"]["falls"] == 1
  assert pe.rank_push(m, standing_rung=0.65)["standing_at_rung"]["rate"] == 0.0


def test_push_commands_conditions():
  cmd, cond = pe.push_commands(8, DT, 300)
  assert cmd.shape == (300, 8, 3)
  assert np.all(cmd[:50] == 0)
  assert cmd[-1, 2, 0] == pytest.approx(1.5) and cmd[-1, 3, 0] == pytest.approx(-1.5)
  assert np.all(cmd[:, cond == 0] == 0)


def _training(n_envs=4096, past=500, parent="runs/p/model_2999.pt"):
  return dict(
    train_num_envs=n_envs,
    train_seed=42,
    iter=2999 + past,
    parent=parent,
    iters_past_parent=past,
    reference_check="ok (default references)",
  )


def _write_suite(path, name, training):
  m = pe.compute_suite_metrics(**_synthetic())
  path.write_text(
    json.dumps(dict(name=name, kind="suite", metrics=m, training=training))
  )


def test_compare_table(tmp_path):
  _write_suite(tmp_path / "a.json", "a", _training())
  _write_suite(tmp_path / "b.json", "b", _training())
  c = dict(pushes=10, falls=1, rate=0.1)
  by_condition = {
    n: {"all": c, "achieved_vx_m_s": v, **{str(m): c for m in pe.PUSH_MAGNITUDES}}
    for n, v in pe.PUSH_CONDITIONS
  }
  push = dict(
    name="a",
    metrics=dict(
      overall=c,
      standing=c,
      moving=c,
      by_magnitude={str(m): c for m in pe.PUSH_MAGNITUDES},
      by_condition=by_condition,
    ),
  )
  (tmp_path / "a_push.json").write_text(json.dumps(push))
  out = tmp_path / "t" / "summary.md"
  header, rows = pe.compare([tmp_path], out)
  text = out.read_text()
  assert text.startswith("Matched table: train_num_envs=4096, iters_past_parent=500")
  assert [r[0] for r in rows] == ["a", "b"]
  assert "push_stand(i)" in header and "rev" in header
  for col in ("train_num_envs", "train_seed", "iter", "parent", "iters_past_parent"):
    assert col in header
  assert rows[0][header.index("parent")] == "p/model_2999"
  assert rows[0][header.index("push(i)")] == pytest.approx(0.1)
  assert rows[1][header.index("push(i)")] is None
  ranked = [i for i, h in enumerate(header) if h.startswith("R:push")]
  informational = [i for i, h in enumerate(header) if h.endswith("(i)")]
  assert len(ranked) == 2 and max(ranked) < min(informational)
  assert rows[0][header.index("R:push_move@0.5")] == pytest.approx(0.1)
  assert rows[0][header.index("vx_run_+1.5")] == pytest.approx(1.5)
  assert "F_runmax_med_BW" in header and "falls<2s_reset" in header
  assert text.count("\n| ") >= 2
  csv_text = out.with_suffix(".csv").read_text().splitlines()
  assert csv_text[0].startswith("name,train_num_envs,train_seed,iter,parent")
  assert len(csv_text) == 3


@pytest.mark.parametrize(
  "other",
  [
    _training(n_envs=8192),  # different batch size
    _training(past=1000),  # different iterations past the parent
    {},  # unknown metadata
  ],
)
def test_compare_refuses_mismatched_runs(tmp_path, other):
  _write_suite(tmp_path / "a.json", "a", _training())
  _write_suite(tmp_path / "b.json", "b", other)
  out = tmp_path / "summary.md"
  with pytest.raises(pe.MismatchError):
    pe.compare([tmp_path], out)
  assert not out.exists()
  header, rows = pe.compare([tmp_path], out, allow_mismatch=True)
  text = out.read_text()
  assert text.startswith("**UNMATCHED TABLE")
  assert header[1] == "mismatch"
  assert rows[0][1] == "-" and rows[1][1] != "-"
  assert out.with_suffix(".csv").read_text().startswith("name,mismatch,")


def _wandb_config(path, args, num_envs=4096, seed=42):
  import yaml

  path.write_text(
    yaml.safe_dump(
      {
        "_wandb": {
          "value": {"e": {"abc": {"args": args, "git": {"commit": "deadbeef"}}}}
        },
        "env_cfg": {"value": {"scene": {"num_envs": num_envs}}},
        "train_cfg": {"value": {"seed": seed}},
      }
    )
  )


def test_training_metadata_wandb_config(tmp_path):
  torch = pytest.importorskip("torch")
  run = tmp_path / "cont"
  run.mkdir()
  ckpt = run / "model_4498.pt"
  torch.save({"iter": 4498}, ckpt)
  _wandb_config(
    run / "config.yaml",
    ["train", "--num-envs", "4096", "--seed", "43", "--iterations", "1500",
     "--checkpoint", "/gone/baseline/model_2999.pt"],
  )  # fmt: skip
  b = pe.training_block(ckpt)
  assert b["iter"] == 4498 and b["iter_source"] == "checkpoint['iter']"
  assert b["train_num_envs"] == 4096 and "wandb CLI args" in b["train_num_envs_source"]
  assert b["train_seed"] == 43  # CLI args win over the cfg value (42)
  assert b["parent"] == "/gone/baseline/model_2999.pt"
  assert b["parent_iter"] == 2999 and "file name" in b["parent_iter_source"]
  assert b["iters_past_parent"] == 1500
  assert b["iters_past_parent_matches_iterations_arg"] is True
  assert b["train_git_revision"] == "deadbeef"
  assert b["reference_check"] == "ok (default references)"
  # From scratch, args lacking --num-envs: cfg value fallback; iter+1.
  scratch = tmp_path / "scratch"
  scratch.mkdir()
  (scratch / "model_2999.pt").write_bytes(b"not a torch file")
  _wandb_config(
    scratch / "config.yaml", ["train", "--reference-dir", "/x/fast_gallop_c"], 2048
  )
  b = pe.training_block(
    scratch / "model_2999.pt", eval_reference_dir="/y/fast_gallop_c"
  )
  assert b["iter"] == 2999 and b["iter_source"] == "file name model_N.pt"
  assert b["train_num_envs"] == 2048 and "cfg values" in b["train_num_envs_source"]
  assert b["parent"] is None and b["iters_past_parent"] == 3000
  assert b["reference_check"] == "ok"
  b = pe.training_block(scratch / "model_2999.pt")
  assert b["reference_check"].startswith("MISMATCH")


def test_training_metadata_provenance_and_overrides(tmp_path):
  torch = pytest.importorskip("torch")
  import yaml

  parent_dir = tmp_path / "parent"
  parent_dir.mkdir()
  torch.save({"iter": 3200}, parent_dir / "model_3200.pt")
  run = tmp_path / "leg"
  (run / "params").mkdir(parents=True)
  torch.save({"iter": 3699}, run / "model_3699.pt")
  (run / "provenance.json").write_text(
    json.dumps(
      dict(num_envs=4096, seed=42, iterations=500, reference_dir=None,
           checkpoint=str(parent_dir / "model_3200.pt"))
    )
  )  # fmt: skip
  (run / "params" / "env.yaml").write_text(yaml.safe_dump({"scene": {"num_envs": 1}}))
  b = pe.training_block(run / "model_3699.pt")
  assert b["train_num_envs"] == 4096 and b["train_num_envs_source"] == "provenance.json"
  assert (
    b["parent_iter"] == 3200 and b["parent_iter_source"] == "parent checkpoint['iter']"
  )
  assert b["iters_past_parent"] == 500
  assert b["iters_past_parent_matches_iterations_arg"] is True
  # Overrides for runs lacking metadata, with their source recorded.
  bare = tmp_path / "bare"
  bare.mkdir()
  torch.save({"iter": 99}, bare / "model_99.pt")
  b = pe.training_block(bare / "model_99.pt")
  assert b["train_num_envs"] is None and b["iters_past_parent"] is None
  assert b["reference_check"].startswith("unknown")
  b = pe.training_block(
    bare / "model_99.pt",
    overrides=dict(train_num_envs=4096, train_seed=7, parent_iter=49),
  )
  assert b["train_num_envs"] == 4096 and b["train_num_envs_source"] == "cli override"
  assert b["iters_past_parent"] == 51 and b["parent_iter_source"] == "cli override"


def test_annotate_existing_json(tmp_path):
  torch = pytest.importorskip("torch")
  run = tmp_path / "run"
  run.mkdir()
  torch.save({"iter": 2999}, run / "model_2999.pt")
  _wandb_config(run / "config.yaml", ["train", "--num-envs", "4096", "--seed", "42"])
  f = tmp_path / "x.json"
  m = pe.compute_suite_metrics(**_synthetic())
  f.write_text(
    json.dumps(
      dict(name="x", kind="suite", metrics=m,
           provenance=dict(checkpoint=str(run / "model_2999.pt"), reference_dir=None))
    )
  )  # fmt: skip
  assert pe.annotate([f], {}) == [str(f)]
  t = json.loads(f.read_text())["training"]
  assert t["iters_past_parent"] == 3000 and t["train_num_envs"] == 4096
  assert "| x | 4096 | 42 | 2999 |" in f.with_suffix(".md").read_text()


def test_parse_batch_entry(tmp_path):
  c, r, n = pe.parse_batch_entry("/a/run1/model_5.pt")
  assert (str(c), r, n) == ("/a/run1/model_5.pt", None, "run1")
  c, r, n = pe.parse_batch_entry("/a/run1/model_5.pt:/refs/g=custom")
  assert (str(c), str(r), n) == ("/a/run1/model_5.pt", "/refs/g", "custom")


def test_drift_drops_windows_ending_in_a_fall():
  T = 200
  h = np.zeros((T, 2))
  h[:, 1] = np.linspace(0, 3.0, T)  # spins before falling
  term = np.zeros((T, 2), bool)
  term[150, 1] = True
  mask = np.ones((T, 2), bool)
  mask[150:, 1] = False
  d = pe.heading_drift(h, mask, DT, term)
  assert d["windows"] == 1 and d["windows_ending_in_fall"] == 1
  assert d["max"] == pytest.approx(0.0)


def test_foot_force_skips_post_reset_drop():
  data = _synthetic()
  # Env 1 (standing) falls at step 400; a 9 BW spawn-drop spike right after reset.
  data["terminated"][400, 1] = True
  data["foot_force"][410, 1, 2] = 9.0 * data["mass_kg"][1] * 9.81
  m = pe.compute_suite_metrics(**data)
  assert m["foot_force_peak_bw"] == pytest.approx(2.0)
  assert m["foot_force_peak_bw_incl_reset_drop"] == pytest.approx(9.0)
  assert m["foot_force_peak_where"]["env"] == 2
  assert m["foot_force_peak_where"]["time_s"] == pytest.approx(2.0)


def test_training_metadata_reads_dump_yaml_python_tags(tmp_path):
  """params/*.yaml from pupper_locomotion train carry python tags (tuples, enums)."""
  torch = pytest.importorskip("torch")
  run = tmp_path / "leg"
  (run / "params").mkdir(parents=True)
  torch.save({"iter": 19}, run / "model_19.pt")
  (run / "params" / "env.yaml").write_text(
    "scene:\n  num_envs: 1024\n  sensors: !!python/tuple\n  - a\n  - b\n"
    "viewer:\n  origin_type: !!python/object/apply:mjlab.viewer.viewer_config.OriginType\n  - 1\n"
  )
  (run / "params" / "agent.yaml").write_text("seed: 43\nclip: !!python/tuple [1, 2]\n")
  (run / "provenance.json").write_text(
    json.dumps({"iterations": 20})
  )  # no num_envs/seed
  meta = pe.read_training_metadata(run / "model_19.pt")
  assert meta["train_num_envs"] == (1024, "params/env.yaml")
  assert meta["train_seed"] == (43, "params/agent.yaml")
