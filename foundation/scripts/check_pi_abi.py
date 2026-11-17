#!/usr/bin/env python3
"""Screen ELF architecture and direct GNU version requirements before transfer.

This does not load candidate code. It checks GLIBC/GLIBCXX/CXXABI requirements
against exact version definitions captured from hashed target providers. It is
not a dependency-closure, ROS API, symbol-implementation or target-load test.
Retain the target-side ldd/runtime identity checks after this screen passes.
Ordinary ELF section tables are required; sectionless/dynamic-only inspection
and extended section numbering are unsupported and refused.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys


SCHEMA = "pupper_pi_abi_target_v1"
PREFIXES = ("GLIBC_", "GLIBCXX_", "CXXABI_")


def load_target(path: Path) -> dict:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError("Target baseline must be an object")
    # The checked-in audit carries the same small standalone target baseline.
    value = value.get("target_abi_baseline", value)
    if not isinstance(value, dict) or value.get("schema") != SCHEMA:
        raise ValueError("Unsupported target baseline schema")
    if value.get("elf_class") not in (32, 64) or value.get("byte_order") not in ("little", "big"):
        raise ValueError("Target needs explicit ELF class and byte order")
    if not isinstance(value.get("machine"), int) or not value.get("providers"):
        raise ValueError("Target needs machine and provider identities")
    for name, provider in value["providers"].items():
        versions = provider.get("provided_versions")
        if (not name or not re.fullmatch(r"[0-9a-f]{64}", provider.get("sha256", ""))
                or not isinstance(versions, list) or not versions
                or any(not isinstance(v, str) or not v.startswith(PREFIXES) for v in versions)):
            raise ValueError("Invalid provider identity/version definitions: " + name)
    return value


def elf_identity(path: Path) -> dict:
    with path.open("rb") as stream:
        header = stream.read(64)
    if (len(header) < 20 or header[:4] != b"\x7fELF" or header[4] not in (1, 2)
            or header[5] not in (1, 2) or header[6] != 1):
        raise ValueError("Invalid or unsupported ELF header")
    wide = header[4] == 2
    endian = "<" if header[5] == 1 else ">"
    if len(header) < (64 if wide else 52):
        raise ValueError("Truncated ELF header")
    section_offset = struct.unpack_from(endian + ("Q" if wide else "I"), header, 40 if wide else 32)[0]
    entry_size, section_count, names_index = struct.unpack_from(endian + "HHH", header, 58 if wide else 46)
    # GNU readelf -V can report "No version information found" with exit zero
    # when a valid loader-visible ELF has had its section table removed. Never
    # interpret that as absence of dynamic version requirements.
    if not section_offset or not section_count or names_index == 0xFFFF:
        raise ValueError("Ordinary ELF section headers required; sectionless/extended-section ELF is unsupported")
    if (entry_size != (64 if wide else 40) or names_index >= section_count
            or section_offset + entry_size * section_count > path.stat().st_size):
        raise ValueError("Invalid or truncated ELF section table")
    return {"elf_class": 32 if header[4] == 1 else 64,
            "byte_order": "little" if header[5] == 1 else "big",
            "machine": struct.unpack_from("<H" if header[5] == 1 else ">H", header, 18)[0]}


def parse_requirements(output: str) -> dict[str, list[str]]:
    """Read *needs*, never the candidate's own exported version definitions."""
    active = False
    recognized = False
    provider = None
    declared_entries = None
    entries = 0
    expected_names = 0
    names = 0
    requirements: dict[str, set[str]] = {}
    def finish_provider():
        if provider is not None and names != expected_names:
            raise ValueError("Incomplete version requirements for " + provider)

    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Version "):
            if active:
                finish_provider()
            active = stripped.startswith("Version needs section")
            recognized |= active
            provider = None
            if active:
                count = re.search(r"contains\s+(\d+)\s+entr(?:y|ies)", stripped)
                if count is None or declared_entries is not None:
                    raise ValueError("Unrecognized or repeated version-needs header")
                declared_entries = int(count.group(1))
        if "No version information found" in line:
            recognized = True
        if not active:
            continue
        match = re.search(r"\bFile:\s+(\S+)", line)
        if match:
            finish_provider()
            provider = match.group(1)
            count = re.search(r"\bCnt:\s+(\d+)", line)
            if count is None:
                raise ValueError("Version provider lacks count")
            expected_names, names = int(count.group(1)), 0
            entries += 1
        match = re.search(r"\bName:\s+(\S+)", line)
        if match:
            if provider is None:
                raise ValueError("Version requirement lacks a provider")
            names += 1
            if match.group(1).startswith(PREFIXES):
                requirements.setdefault(provider, set()).add(match.group(1))
    finish_provider()
    if declared_entries is not None and entries != declared_entries:
        raise ValueError("Incomplete version-needs provider list")
    if not recognized:
        raise ValueError("Unrecognized readelf version output; refusing an empty success")
    return {name: sorted(versions) for name, versions in sorted(requirements.items())}


def version_output(path: Path, *, reader: str | None, docker_image: str | None) -> str:
    if reader:
        command = [reader, "--wide", "--version-info", str(path)]
    elif docker_image:
        # Explicit opt-in, cached image only, no network or devices, input read-only.
        if "," in str(path.parent):
            raise ValueError("Docker mount path contains a comma; use a local readelf")
        command = ["docker", "run", "--rm", "--pull=never", "--network=none", "--read-only",
                   "--cap-drop=ALL", "--security-opt=no-new-privileges",
                   "--env", "LC_ALL=C",
                   "--mount", f"type=bind,src={path.parent},dst=/candidate,readonly",
                   "--entrypoint", "/usr/bin/readelf", docker_image,
                   "--wide", "--version-info", "/candidate/" + path.name]
    else:
        raise ValueError("No GNU readelf found; install binutils or pass --docker-image for a cached test image")
    result = subprocess.run(command, capture_output=True, text=True, timeout=45,
                            env={**os.environ, "LC_ALL": "C"})
    if result.returncode or result.stderr.strip():
        raise ValueError("readelf failed: " + result.stderr.strip()[:2000])
    return result.stdout


def check_file(path: Path, target: dict, *, reader=None, docker_image=None) -> dict:
    path = path.resolve(strict=True)
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    identity = elf_identity(path)
    mismatches = {key: {"actual": identity[key], "target": target[key]}
                  for key in identity if identity[key] != target[key]}
    result = {"path": str(path), "sha256": digest, **identity,
              "architecture_mismatches": mismatches, "missing_versions": {}}
    if mismatches:
        result["passed"] = False
        return result
    required = parse_requirements(version_output(path, reader=reader, docker_image=docker_image))
    missing = {}
    for provider, versions in required.items():
        available = target["providers"].get(provider, {}).get("provided_versions", [])
        absent = sorted(set(versions) - set(available))
        if absent:
            missing[provider] = absent
    # Guard against an accidentally edited input while the reader was running.
    with path.open("rb") as stream:
        if hashlib.file_digest(stream, "sha256").hexdigest() != digest:
            raise ValueError("ELF changed while inspecting it")
    result.update(required_versions=required, missing_versions=missing, passed=not missing)
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, type=Path,
                        help="standalone target baseline, or build audit containing target_abi_baseline")
    parser.add_argument("--readelf", help="GNU readelf executable (otherwise discover readelf/greadelf)")
    parser.add_argument("--docker-image", help="explicit cached image fallback when no local readelf is installed")
    parser.add_argument("--report", type=Path, help="optional NEW JSON receipt path; existing files refused")
    parser.add_argument("elf", nargs="+", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.report and args.report.exists():
            raise ValueError("Report exists; choose a new path")
        target = load_target(args.target)
        reader = args.readelf or shutil.which("readelf") or shutil.which("greadelf")
        results = []
        for path in args.elf:
            try:
                results.append(check_file(path, target, reader=reader, docker_image=args.docker_image))
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                results.append({"path": str(path), "passed": False, "error": str(error)})
        passed = all(r["passed"] for r in results)
        receipt = {"schema": "pupper_pi_abi_screen_v1", "passed": passed,
                   "target_baseline_sha256": hashlib.sha256(json.dumps(target, sort_keys=True).encode()).hexdigest(),
                   "target_observed_utc": target.get("observed_utc"), "files": results,
                   "scope": "ELF architecture and direct GLIBC/GLIBCXX/CXXABI version-definition membership only",
                   "limitations": ["Does not prove transitive dependency closure, individual symbol implementations, ROS ABI or loadability.",
                                   "Provider hashes identify the captured baseline; current target still requires ldd and runtime identity checks.",
                                   "Requires ordinary section tables; sectionless ELF and extended section numbering are refused.",
                                   "Other version namespaces and unversioned dependencies are outside this screen."]}
        text = json.dumps(receipt, indent=2) + "\n"
        if args.report:
            with args.report.open("x") as stream:
                stream.write(text)
        sys.stdout.write(text)
        return 0 if passed else 2 if any("error" in r for r in results) else 1
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, "ABI screen refused: " + str(error) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
