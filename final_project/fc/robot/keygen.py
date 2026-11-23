"""Create the shared laptop<->Pi link key: `python -m fc.robot.keygen [--force] [--out PATH]`.

Writes 32 random bytes as hex to fc/config/link.key (gitignored) with mode 0600.
`fc/robot/stage.sh <host>` copies it to the Pi as ~/fc-v1/robot/link.key.
Replacing the key means re-staging every robot.
"""
from __future__ import annotations

import argparse
import os
import secrets
import sys
from pathlib import Path

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "config" / "link.key"


def write_key(path: Path, force: bool = False) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | (os.O_TRUNC if force else os.O_EXCL)
    fd = os.open(path, flags, 0o600)
    try:
        os.fchmod(fd, 0o600)
        os.write(fd, (secrets.token_hex(32) + "\n").encode())
    finally:
        os.close(fd)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--force", action="store_true", help="replace an existing key (re-stage every robot)")
    args = parser.parse_args(argv)
    try:
        path = write_key(args.out, args.force)
    except FileExistsError:
        print(f"{args.out} exists; keeping it (use --force to replace, then re-stage every robot)", file=sys.stderr)
        return 1
    print(f"wrote {path} (0600). Next: fc/robot/stage.sh <host> for each robot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
