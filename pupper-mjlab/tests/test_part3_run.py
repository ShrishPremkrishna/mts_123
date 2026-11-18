"""Part 3 runbook decision logic (pure; no GPU): between-leg gates, lockstep extension, matched selection."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "opt_lab_2"))

import gpu_slot  # noqa: E402
import part3_run as pr  # noqa: E402


def pm(flat=40, bumpy=80, nompd=90, seam="PASS"):
  return dict(flat_dr=flat, bumpy_dr=bumpy, nompd=nompd, seam_flat=seam)


def test_gate_invalid_and_g2_stop():
  base = pm(40, 80)
  assert pr.gate_after_leg(2, base, pm(), ["overflow"])["action"] == "stop"
  # G2 fires only at +500: flat_dr doubled and bumpy_dr no better.
  assert pr.gate_after_leg(1, base, pm(80, 80), [])["action"] == "stop"
  assert pr.gate_after_leg(1, base, pm(80, 70), [])["action"] == "continue"
  assert pr.gate_after_leg(1, base, pm(79, 90), [])["action"] == "continue"
  assert pr.gate_after_leg(2, base, pm(80, 80), [])["action"] == "continue"


def test_extension_lockstep_rule_and_cap():
  base = pm(40, 80)
  due, _ = pr.extension_due(base, pm(45, 90), pm(48, 75), 1500)
  assert due  # bumpy_dr dropped 15 > 10, flat_dr within 10 of +0
  assert not pr.extension_due(base, pm(45, 90), pm(55, 75), 1500)[0]  # flat_dr drifted 15
  assert not pr.extension_due(base, pm(45, 90), pm(48, 82), 1500)[0]  # dropped only 8
  assert not pr.extension_due(base, pm(45, 90), pm(48, 75), 3000)[0]  # cap


def test_regressions():
  assert pr.regressions(pm(55, 80, 90), pm(40, 80, 90)) == ["flat_dr"]
  assert pr.regressions(pm(40, 80, 101), pm(40, 80, 90)) == ["nompd"]
  assert pr.regressions(pm(seam="FAIL"), pm(seam="PASS")) == ["seam_flat"]
  assert pr.regressions(pm(50, 90, 100), pm(40, 80, 90)) == []  # +10 each is within the band


def test_matched_selection_requires_both_seeds():
  keys = ["+500", "+1000", "+1500"]
  ev1 = {"+500": pm(), "+1000": pm(40, 80), "+1500": pm(55, 80)}
  calls = []

  def ev2_same(key):
    calls.append(key)
    return {"+1000": pm(40, 80), "+1500": pm(60, 80)}[key]

  r = pr.matched_selection(keys, ev1, ev2_same)
  assert r["selected"] == "+1000" and "BOTH" in r["notes"][0]
  r = pr.matched_selection(keys, ev1, lambda key: {"+1000": pm(40, 80), "+1500": pm(45, 80)}[key])
  assert r["selected"] == "+1500" and "did not replicate" in r["notes"][0]
  calls.clear()
  r = pr.matched_selection(keys, {"+1000": pm(), "+1500": pm()}, ev2_same)
  assert r["selected"] == "+1500" and calls == []  # no seed-4321 evals unless seed 1234 regresses
  assert pr.matched_selection(["+50"], {"+50": pm()}, ev2_same)["selected"] == "+50"


def test_gpu_slot_cap_reads_concurrency_file(tmp_path, monkeypatch):
  f = tmp_path / "CONCURRENCY"
  f.write_text("3\n# max concurrent GPU jobs\n")
  monkeypatch.setattr(gpu_slot, "CONCURRENCY_FILE", f)
  assert gpu_slot.cap() == 3
  monkeypatch.setattr(gpu_slot, "CONCURRENCY_FILE", tmp_path / "missing")
  assert gpu_slot.cap() == gpu_slot.DEFAULT_CAP


def test_gpu_slot_launch_holds_shared_lock(tmp_path, monkeypatch):
  """launch() takes the flock W3 uses and launches only below the cap."""
  import fcntl
  import threading

  lock = tmp_path / "launch.lock"
  monkeypatch.setattr(gpu_slot, "LAUNCH_LOCK", lock)
  monkeypatch.setattr(gpu_slot, "REGISTER_WAIT_S", 0)
  counts = iter([4, 4, 1])  # full, full, then a free slot
  monkeypatch.setattr(gpu_slot, "gpu_procs", lambda: next(counts, 1))
  monkeypatch.setattr(gpu_slot, "cap", lambda: 4)
  p = gpu_slot.launch([sys.executable, "-c", "pass"], poll=0.01, log=lambda m: None)
  assert p.wait() == 0
  # While another holder has the lock, launch() blocks.
  lock.touch()
  started = threading.Event()
  with open(lock) as lk:
    fcntl.flock(lk, fcntl.LOCK_EX)
    t = threading.Thread(target=lambda: (gpu_slot.launch([sys.executable, "-c", "pass"], poll=0.01, log=lambda m: None).wait(),
                                         started.set()))
    t.start()
    t.join(0.3)
    assert not started.is_set()
  t.join(5)
  assert started.is_set()
