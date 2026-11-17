"""Regression checks for pre-transfer ABI screening; never execute candidate ELF."""
import importlib.util
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tarfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("pi_abi", ROOT / "foundation/scripts/check_pi_abi.py")
abi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(abi)
ARTIFACTS = ROOT.parent / "pupper-foundation-artifacts/2026-11-16/movement-offline"
TARGET = ROOT / "foundation/results/build-environment-audit-20261116.json"
IMAGE = "cs123-ros-jazzy-slew-check:latest"


def header(path, machine=183):
    data = bytearray(128)
    data[:7] = b"\x7fELF\x02\x01\x01"
    struct.pack_into("<H", data, 18, machine)
    struct.pack_into("<Q", data, 40, 64)
    struct.pack_into("<HHH", data, 58, 64, 1, 0)
    path.write_bytes(data)
    return path


def test_definitions_do_not_satisfy_needs_and_namespaces_keep_their_provider():
    output = """Version definition section '.gnu.version_d' contains 1 entry:
      0x00: Name: GLIBC_2.38
Version needs section '.gnu.version_r' contains 2 entries:
      Version: 1 File: libc.so.6 Cnt: 1
      Name: GLIBC_2.38 Flags: none Version: 4
      Version: 1 File: libgcc_s.so.1 Cnt: 1
      Name: GCC_3.0 Flags: none Version: 5
"""
    assert abi.parse_requirements(output) == {"libc.so.6": ["GLIBC_2.38"]}


@pytest.mark.parametrize("output", [
    "Version needs section '.gnu.version_r' contains 2 entries:\n",
    "Version needs section '.gnu.version_r' contains 1 entry:\nFile: libc.so.6 Cnt: 2\nName: GLIBC_2.17\n",
    "unexpected reader output",
])
def test_incomplete_reader_output_cannot_pass(output):
    with pytest.raises(ValueError):
        abi.parse_requirements(output)


def test_wrong_machine_fails_before_reader(tmp_path, monkeypatch):
    target = abi.load_target(TARGET)
    monkeypatch.setattr(abi, "version_output", lambda *a, **k: pytest.fail("reader must not run"))
    result = abi.check_file(header(tmp_path / "wrong.so", machine=62), target)
    assert not result["passed"]
    assert result["architecture_mismatches"]["machine"] == {"actual": 62, "target": 183}


@pytest.mark.parametrize("offset,fmt", [(40, "<Q"), (60, "<H")])
def test_missing_section_table_refused_before_reader(tmp_path, monkeypatch, offset, fmt):
    path = header(tmp_path / "sectionless.so")
    data = bytearray(path.read_bytes())
    struct.pack_into(fmt, data, offset, 0)
    path.write_bytes(data)
    monkeypatch.setattr(abi, "version_output", lambda *a, **k: pytest.fail("reader must not run"))
    with pytest.raises(ValueError, match="Ordinary ELF section headers required"):
        abi.check_file(path, abi.load_target(TARGET))


def test_exact_provider_membership_not_just_maximum(tmp_path, monkeypatch):
    target = abi.load_target(TARGET)
    target["providers"]["libc.so.6"]["provided_versions"] = ["GLIBC_2.17", "GLIBC_2.36"]
    target["providers"]["libm.so.6"]["provided_versions"].append("GLIBC_2.34")
    monkeypatch.setattr(abi, "version_output", lambda *a, **k:
        "Version needs section '.gnu.version_r' contains 1 entry:\nFile: libc.so.6 Cnt: 1\nName: GLIBC_2.34\n")
    result = abi.check_file(header(tmp_path / "gap.so"), target)
    assert result["missing_versions"] == {"libc.so.6": ["GLIBC_2.34"]}


def test_exit_zero_reader_warning_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(abi.subprocess, "run", lambda *a, **k:
        subprocess.CompletedProcess([], 0, "No version information found in this file.", "readelf: Warning: corrupt version data"))
    with pytest.raises(ValueError, match="corrupt"):
        abi.version_output(tmp_path / "candidate", reader="readelf", docker_image=None)


@pytest.fixture(scope="module")
def real_inputs(tmp_path_factory):
    archive = ARTIFACTS / "movement-20261116-v2.tar.gz"
    if not archive.exists() or not (ARTIFACTS / "release-v1").exists():
        pytest.skip("Preserved v1/v2 artifacts are not available on this machine")
    reader = shutil.which("readelf") or shutil.which("greadelf")
    docker = None
    if not reader:
        if not shutil.which("docker") or subprocess.run(
                ["docker", "image", "inspect", IMAGE], capture_output=True, timeout=10).returncode:
            pytest.skip("No local GNU readelf or cached Docker reader")
        docker = IMAGE
    target = abi.load_target(TARGET)
    destination = tmp_path_factory.mktemp("pi-abi-elf")
    paths = {
        "driver": "prefix/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so",
        "controller": "prefix/neural_controller/lib/libneural_controller.so",
        "gateway": "prefix/joy_utils/lib/joy_utils/estop_controller",
        "mux": "prefix/cmd_vel_mux/lib/cmd_vel_mux/cmd_vel_mux_node",
    }
    with tarfile.open(archive) as tar:
        toolchain = json.load(tar.extractfile("build-evidence/target-toolchain.json"))
        for role, soname in (("libc", "libc.so.6"), ("libstdc++", "libstdc++.so.6")):
            assert target["providers"][soname]["sha256"] == toolchain["libraries"][role]["sha256"]
        for role, relative in paths.items():
            # Copy only named regular ELF payloads into test scratch; never modify releases.
            member = tar.getmember("movement-20261116-v2/" + relative)
            assert member.isfile()
            (destination / role).write_bytes(tar.extractfile(member).read())
    return target, paths, destination, reader, docker


@pytest.mark.parametrize("role", ["driver", "controller", "gateway", "mux"])
def test_real_release_versions_and_preserved_driver(real_inputs, role):
    target, paths, destination, reader, docker = real_inputs
    old = abi.check_file(ARTIFACTS / "release-v1" / paths[role], target,
                         reader=reader, docker_image=docker)
    new = abi.check_file(destination / role, target, reader=reader, docker_image=docker)
    if role == "driver":
        assert old["passed"]
        assert old["sha256"] == new["sha256"] == "1dd92b65851bf9dae53aac28b3e20f6f81c70edadb28dd84dcbadcb9606ff429"
    else:
        assert not old["passed"], old
        assert old["missing_versions"], old
    if role == "controller":
        assert "GLIBC_2.38" in old["missing_versions"]["libc.so.6"]
        assert "GLIBCXX_3.4.32" in old["missing_versions"]["libstdc++.so.6"]
    assert new["passed"], new
    assert new["required_versions"]


def test_real_v1_sectionless_elf_cannot_false_pass(real_inputs, tmp_path, capsys):
    target, paths, _, reader, docker = real_inputs
    original = (ARTIFACTS / "release-v1" / paths["controller"]).read_bytes()
    data = bytearray(original)
    struct.pack_into("<Q", data, 40, 0)  # e_shoff
    struct.pack_into("<HH", data, 60, 0, 0)  # e_shnum, e_shstrndx
    path = tmp_path / "sectionless-v1.so"
    path.write_bytes(data)
    assert data[64:] == original[64:]  # Program headers and dynamic payload unchanged.
    output = abi.version_output(path, reader=reader, docker_image=docker)
    assert "No version information found" in output  # Reproduce GNU readelf's false-empty result.
    with pytest.raises(ValueError, match="Ordinary ELF section headers required"):
        abi.check_file(path, target, reader=reader, docker_image=docker)
    assert abi.main(["--target", str(TARGET), str(path)]) == 2
    assert json.loads(capsys.readouterr().out)["passed"] is False
