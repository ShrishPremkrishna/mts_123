#!/usr/bin/env python3
"""CLI wrapper: run one command through gpu_slot (W3's launch lock + ~/recovery/CONCURRENCY cap).

  python scripts/opt_lab_2/gpu_slot_run.py -- <command ...>
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gpu_slot  # noqa: E402

if __name__ == "__main__":
  argv = sys.argv[1:]
  if argv[:1] == ["--"]:
    argv = argv[1:]
  if not argv:
    sys.exit(__doc__)
  sys.exit(gpu_slot.run(argv, log=lambda m: print(m, flush=True)))
