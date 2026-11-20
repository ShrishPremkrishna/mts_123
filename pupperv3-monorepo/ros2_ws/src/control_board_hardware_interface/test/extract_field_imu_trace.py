"""Export original receipt times and raw SH-2 payloads for test_bno_cache archive."""
import argparse
import hashlib
import json
import tarfile
from pathlib import Path

ARCHIVE_SHA256 = "af16ba0992b0563e46387630935ef5fd10119cd86d4ed42640c7d6de666798dd"
MEMBER = "sensor-session/remote/comparison-02/packets.jsonl"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.archive.open("rb") as stream:
        if hashlib.sha256(stream.read()).hexdigest() != ARCHIVE_SHA256:
            parser.error("source differs from the recorded comparison-02 archive")
    with tarfile.open(args.archive) as archive, args.output.open("w") as output:
        for line in archive.extractfile(MEMBER):
            row = json.loads(line)
            if row.get("channel") == 3:
                print(round(row["monotonic_end_s"] * 1e9), row["payload_hex"], file=output)


if __name__ == "__main__":
    main()
