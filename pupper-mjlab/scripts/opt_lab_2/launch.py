#!/usr/bin/env python3
"""Reproducible launcher for Opt Lab 2 training runs.

Wraps ``python -m mjlab.scripts.pupper_locomotion train`` and

* refuses to run unless every file under ``--reference-dir`` and the ``--weights``
  JSON is tracked, unmodified, and last changed by a commit that is already on
  ``origin/markus/opt-lab-2`` (so the run can be rebuilt from the remote);
* records the git SHA, reference dir, weights path and per-file blob/commit/sha256 in
  the WandB run config (key ``opt_lab_2``) and notes, tags the run ``rp``;
* uploads the reference dir (npz + manifest) and the weights JSON as WandB artifacts
  (``use_artifact``, so they show as run inputs);
* names runs ``rp-<name>`` by default (the WandB run name is the ``--out`` basename);
* adds the deployment physics flags by default and passes every other flag through
  (``--checkpoint``, ``--iterations``, ``--seed``, ``--logger``, ...);
* tees stdout/stderr to ``<out>.log`` and writes ``<out>.launch.json`` next to ``--out``.

Usage (from ``pupper-mjlab/``)::

    uv run --no-sync python scripts/opt_lab_2/launch.py --out ~/runs/opt-lab-2/gallop_c-s42 \\
        --reference-dir configs/opt-lab-2/gallop_c --weights configs/opt-lab-2/weights/x.json \\
        --iterations 3000 --seed 42

See scripts/opt_lab_2/README.md.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REMOTE = "origin"
BRANCH = "markus/opt-lab-2"
ORIGIN_REF = f"{REMOTE}/{BRANCH}"
MJLAB_ROOT = Path(__file__).resolve().parents[2]  # pupper-mjlab/
DRIVER_LIMITS = (MJLAB_ROOT.parent / "foundation/configs/class-driver-joint-limits.json").resolve()
PHYSICS_FLAGS = ["--deployment-target-clamp", "--compose-com-randomization", "--locomotion-torso-contact"]
META_ENV = "OPT_LAB_2_LAUNCH_META"


TOPLEVEL = Path(subprocess.run(["git", "-C", str(MJLAB_ROOT), "rev-parse", "--show-toplevel"], capture_output=True,
                               text=True, check=True).stdout.strip())


def git(*args: str, check: bool = True) -> str:
  """Run git at the repo top level (all paths passed in are top-level relative or absolute)."""
  r = subprocess.run(["git", "-C", str(TOPLEVEL), *args], capture_output=True, text=True)
  if check and r.returncode != 0:
    raise RuntimeError(f"git {' '.join(args)} failed: {r.stderr.strip()}")
  return r.stdout.strip()


def sha256(path: Path) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 20), b""):
      h.update(chunk)
  return h.hexdigest()


def is_ancestor(commit: str, ref: str) -> bool:
  return subprocess.run(["git", "-C", str(TOPLEVEL), "merge-base", "--is-ancestor", commit, ref]).returncode == 0


def check_pinned(path: Path, toplevel: Path) -> tuple[list[dict], list[str]]:
  """Return (file records, errors) for a file or every file in a directory."""
  errors: list[str] = []
  try:
    rel_root = path.relative_to(toplevel)
  except ValueError:
    return [], [f"{path} is outside the git repo {toplevel}"]
  status = git("status", "--porcelain", "--untracked-files=all", "--", str(rel_root))
  if status:
    errors.append(f"{path} has untracked/modified/staged changes:\n    " + status.replace("\n", "\n    "))
  files = sorted(p for p in path.rglob("*") if p.is_file()) if path.is_dir() else [path]
  if not files:
    errors.append(f"{path} contains no files")
  records = []
  for f in files:
    rel = str(f.relative_to(toplevel))
    if subprocess.run(["git", "-C", str(toplevel), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
      errors.append(f"{rel} is not tracked by git")
      continue
    commit = git("log", "-1", "--format=%H", "HEAD", "--", rel)
    if not commit or not is_ancestor(commit, ORIGIN_REF):
      errors.append(f"{rel}: last commit {commit[:10] or '?'} is not on {ORIGIN_REF} (push it first)")
    records.append(dict(path=rel, commit=commit, blob=git("rev-parse", f"HEAD:{rel}"),
                        sha256=sha256(f), bytes=f.stat().st_size))
  return records, errors


def flag_value(argv: list[str], flag: str) -> str | None:
  for i, a in enumerate(argv):
    if a == flag and i + 1 < len(argv):
      return argv[i + 1]
    if a.startswith(flag + "="):
      return a.split("=", 1)[1]
  return None


# --------------------------------------------------------------------------- child


def child_main(train_argv: list[str]) -> None:
  """Runs inside the training process: hook WandB init, then run the stock CLI."""
  meta = json.loads(Path(os.environ[META_ENV]).read_text())
  from rsl_rl.utils import wandb_log_writer

  original_init = wandb_log_writer.WandbLogWriter.__init__

  def hooked_init(self, log_dir, project_name, *a, **kw):
    original_init(self, log_dir, project_name, *a, **kw)
    import wandb

    run = wandb.run
    run.config.update({"opt_lab_2": meta}, allow_val_change=True)
    run.tags = tuple(dict.fromkeys([*(run.tags or ()), "rp", *meta.get("tags", [])]))
    if meta.get("group"):
      try:
        run.group = meta["group"]  # only honored by some wandb versions after init
      except Exception:
        pass
    run.notes = meta["notes"]
    for kind, key in (("reference", "reference_dir"), ("weights", "weights")):
      if not meta.get(key):
        continue
      src = Path(meta["repo_toplevel"]) / meta[key]
      name = f"{kind}-" + "".join(c if c.isalnum() or c in "-_." else "_" for c in Path(meta[key]).name)
      art = wandb.Artifact(name, type=kind, metadata=dict(git_path=meta[key], head_sha=meta["head_sha"],
                                                         files=meta[f"{key}_files"]))
      if src.is_dir():
        art.add_dir(str(src))
      else:
        art.add_file(str(src))
      try:
        run.use_artifact(art)
      except Exception as e:  # older wandb: fall back to an output artifact
        print(f"[launch] use_artifact failed ({e}); using log_artifact", flush=True)
        run.log_artifact(art)
      print(f"[launch] logged WandB artifact {name} from {meta[key]}", flush=True)
    Path(log_dir, "launch.json").write_text(json.dumps(meta, indent=2) + "\n")

  wandb_log_writer.WandbLogWriter.__init__ = hooked_init
  from mjlab.scripts import pupper_locomotion

  sys.argv = ["pupper_locomotion", "train", *train_argv]
  pupper_locomotion.main()


# --------------------------------------------------------------------------- parent


def main() -> int:
  if len(sys.argv) > 1 and sys.argv[1] == "--_child":
    child_main(sys.argv[2:])
    return 0

  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--out", type=Path, help="Run dir (must not exist). Basename gets an 'rp-' prefix unless --no-prefix.")
  p.add_argument("--name", help="Run name; with --runs-root instead of --out")
  p.add_argument("--runs-root", type=Path, default=Path.home() / "runs/opt-lab-2")
  p.add_argument("--reference-dir", type=Path)
  p.add_argument("--weights", type=Path)
  p.add_argument("--no-prefix", action="store_true", help="Do not prefix the run name with 'rp-'")
  p.add_argument("--no-default-physics", action="store_true", help="Do not add the deployment physics flags")
  p.add_argument("--tags", default="", help="Comma-separated extra WandB tags")
  p.add_argument("--group", default=None, help="WandB run group (via WANDB_RUN_GROUP)")
  p.add_argument("--notes", default="", help="Free text prepended to the WandB notes")
  p.add_argument("--dry-run", action="store_true", help="Run checks, print the command, do not train")
  p.add_argument("--quiet", action="store_true", help="Log to file only (no terminal echo)")
  args, passthrough = p.parse_known_args()
  if passthrough[:1] == ["--"]:
    passthrough = passthrough[1:]
  for banned in ("--out", "--reference-dir", "--weights"):
    if flag_value(passthrough, banned) is not None:
      p.error(f"{banned} must be given to the launcher itself")

  if args.out is None:
    if not args.name:
      p.error("give --out or --name")
    args.out = args.runs_root / args.name
  out = args.out.expanduser().resolve()
  if not args.no_prefix and not out.name.startswith("rp-"):
    out = out.with_name("rp-" + out.name)
  if out.exists():
    p.error(f"{out} already exists")

  toplevel = TOPLEVEL
  fetch = subprocess.run(["git", "-C", str(MJLAB_ROOT), "fetch", "-q", REMOTE, BRANCH], capture_output=True, text=True)
  if fetch.returncode:
    print(f"[launch] WARNING: git fetch failed ({fetch.stderr.strip()}); using cached {ORIGIN_REF}", file=sys.stderr)

  errors: list[str] = []
  meta: dict = dict(launcher="scripts/opt_lab_2/launch.py", launched_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                    head_sha=git("rev-parse", "HEAD"), origin_ref=ORIGIN_REF,
                    origin_sha=git("rev-parse", ORIGIN_REF), repo_toplevel=str(toplevel), host=os.uname().nodename)
  meta["head_on_origin"] = is_ancestor(meta["head_sha"], ORIGIN_REF)
  src_dirty = git("status", "--porcelain", "--", str(MJLAB_ROOT / "src"))
  meta["src_dirty"] = bool(src_dirty)
  for key in ("reference_dir", "weights"):
    path = getattr(args, key)
    meta[key] = None
    if path is None:
      continue
    path = path.expanduser().resolve()
    if not path.exists():
      errors.append(f"--{key.replace('_', '-')} {path} does not exist")
      continue
    if key == "reference_dir" and not path.is_dir():
      errors.append(f"--reference-dir {path} is not a directory")
      continue
    records, errs = check_pinned(path, toplevel)
    errors += errs
    meta[key] = str(path.relative_to(toplevel)) if path.is_relative_to(toplevel) else str(path)
    meta[f"{key}_files"] = records
  if errors:
    print("[launch] REFUSING to launch (commit and push the inputs to %s first):" % ORIGIN_REF, file=sys.stderr)
    for e in errors:
      print("  - " + e, file=sys.stderr)
    return 2
  if not meta["head_on_origin"]:
    print(f"[launch] WARNING: HEAD {meta['head_sha'][:10]} is not on {ORIGIN_REF}; code may not be reproducible", file=sys.stderr)
  if src_dirty:
    print("[launch] WARNING: uncommitted changes under pupper-mjlab/src:\n" + src_dirty, file=sys.stderr)

  train_argv = ["--out", str(out)]
  if args.reference_dir:
    train_argv += ["--reference-dir", str(toplevel / meta["reference_dir"])]
  if args.weights:
    train_argv += ["--weights", str(toplevel / meta["weights"])]
  if not args.no_default_physics:
    train_argv += [f for f in PHYSICS_FLAGS if f not in passthrough]
    if flag_value(passthrough, "--driver-limits") is None:
      train_argv += ["--driver-limits", str(DRIVER_LIMITS)]
  # Resolve a relative --checkpoint against the caller's cwd (the child runs in pupper-mjlab/).
  passthrough = list(passthrough)
  for i, a in enumerate(passthrough[:-1]):
    if a in ("--checkpoint", "--driver-limits"):
      passthrough[i + 1] = str(Path(passthrough[i + 1]).expanduser().resolve())
  train_argv += passthrough
  ckpt = flag_value(train_argv, "--checkpoint")
  if ckpt:
    meta["checkpoint"] = ckpt
    meta["checkpoint_sha256"] = sha256(Path(ckpt))
  meta["run_name"] = out.name
  meta["train_argv"] = train_argv
  meta["tags"] = [t for t in args.tags.split(",") if t]
  meta["group"] = args.group
  meta["notes"] = (args.notes + "\n" if args.notes else "") + (
      f"git {meta['head_sha'][:10]} (on origin: {meta['head_on_origin']}, src dirty: {meta['src_dirty']}); "
      f"reference_dir={meta['reference_dir'] or 'none (stock trot fallback in fast slot)'}; "
      f"weights={meta['weights'] or 'profile default'}; checkpoint={ckpt or 'none'}")

  cmd = [sys.executable, str(Path(__file__).resolve()), "--_child", *train_argv]
  print("[launch] " + " ".join(cmd[3:]), flush=True)
  if args.dry_run:
    print(json.dumps(meta, indent=2))
    return 0

  out.parent.mkdir(parents=True, exist_ok=True)
  meta_path = out.with_name(out.name + ".launch.json")
  log_path = out.with_name(out.name + ".log")
  meta_path.write_text(json.dumps(meta, indent=2) + "\n")
  env = dict(os.environ, PYTHONUNBUFFERED="1", **{META_ENV: str(meta_path)})
  env.setdefault("MUJOCO_GL", "egl")
  if args.group:
    env["WANDB_RUN_GROUP"] = args.group
  print(f"[launch] log: {log_path}", flush=True)
  with open(log_path, "ab") as log:
    log.write(f"[launch] {json.dumps(meta)}\n".encode())
    proc = subprocess.Popen(cmd, cwd=MJLAB_ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    signal.signal(signal.SIGTERM, lambda s, f: proc.send_signal(signal.SIGTERM))
    try:
      for line in iter(proc.stdout.readline, b""):
        log.write(line)
        log.flush()
        if not args.quiet:
          sys.stdout.buffer.write(line)
          sys.stdout.flush()
    except KeyboardInterrupt:
      proc.send_signal(signal.SIGINT)
    rc = proc.wait()
    log.write(f"[launch] exit code {rc}\n".encode())
  print(f"[launch] exit code {rc}; log {log_path}", flush=True)
  return rc


if __name__ == "__main__":
  sys.exit(main())
