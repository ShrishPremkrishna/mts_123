"""provision/clone_live.sh guards and plan, run against fake lsblk/findmnt/df/du and a temp source root.
No disk is touched: destructive tools (wipefs, sfdisk, mkfs, mount, rsync, ...) are fakes that only record calls,
and every test asserts none of them ran."""
import json
import os
import stat
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "provision" / "clone_live.sh"
GB = 1_000_000_000
BOOT_SERIAL, TARGET_SERIAL = "03025211111625224118", "03025113111625215409"
DESTRUCTIVE = ["wipefs", "sfdisk", "mkfs.vfat", "mkfs.ext4", "mount", "umount", "rsync", "partprobe",
               "udevadm", "blkid", "ssh-keygen"]

FAKE_LSBLK = r'''
import json, os, sys
fx = json.load(open(os.environ["FAKE_DISKS"]))
devs = fx["devices"]
flags, cols, pos, a = set(), [], [], sys.argv[1:]
i = 0
while i < len(a):
    x = a[i]
    if x.startswith("-"):
        for c in x[1:]:
            if c == "o":
                i += 1; cols = a[i].split(",")
            else:
                flags.add(c)
    else:
        pos.append(x)
    i += 1
if pos:
    rows = [d for d in devs if d["NAME"] == pos[0]]
    if not rows:
        sys.exit("lsblk: %s: not a block device" % pos[0])
    if "d" not in flags:
        rows += [d for d in devs if d.get("PKNAME") == pos[0]]
else:
    rows = [d for d in devs if "d" not in flags or not d.get("PKNAME")]
for r in rows:
    out = []
    for c in cols:
        v = r.get(c, "")
        if c in ("NAME", "PKNAME") and "p" not in flags and v:
            v = v.rsplit("/", 1)[-1]
        out.append(str(v))
    print(" ".join(out))
'''
FAKE_FINDMNT = r'''
import json, os, sys
fx = json.load(open(os.environ["FAKE_DISKS"]))
src = fx["findmnt"].get(sys.argv[-1])
if not src:
    sys.exit(1)
print(src)
'''
FAKE_DF = r'''
import json, os, sys
fx = json.load(open(os.environ["FAKE_DISKS"]))
path = sys.argv[-1].rstrip("/")
print("       Used")
print(fx["df"]["boot"] if path.endswith("boot/firmware") else fx["df"]["root"])
'''
FAKE_DU = r'''
import os, sys
p = sys.argv[-1]
tot = 0
for d, _, fs in os.walk(p):
    tot += sum(os.lstat(os.path.join(d, f)).st_size for f in fs)
if os.path.isfile(p):
    tot = os.lstat(p).st_size
print("%d\t%s" % (tot, p))
'''


def disks(**over):
    fx = {
        "devices": [
            {"NAME": "/dev/sda", "TYPE": "disk", "SERIAL": BOOT_SERIAL, "MODEL": "SanDisk 3.2Gen1",
             "SIZE": 128 * GB, "TRAN": "usb"},
            {"NAME": "/dev/sda1", "TYPE": "part", "PKNAME": "/dev/sda", "MOUNTPOINT": "/boot/firmware"},
            {"NAME": "/dev/sda2", "TYPE": "part", "PKNAME": "/dev/sda", "MOUNTPOINT": "/"},
            {"NAME": "/dev/sdb", "TYPE": "disk", "SERIAL": TARGET_SERIAL, "MODEL": "SanDisk 3.2Gen1",
             "SIZE": 128 * GB, "TRAN": "usb"},
            {"NAME": "/dev/sdb1", "TYPE": "part", "PKNAME": "/dev/sdb", "MOUNTPOINT": ""},
            {"NAME": "/dev/loop0", "TYPE": "loop", "SERIAL": "", "SIZE": 0},
        ],
        "findmnt": {"/": "/dev/sda2", "/boot/firmware": "/dev/sda1"},
        "df": {"root": 40 * GB, "boot": 80_000_000},
    }
    fx.update(over)
    return fx


@pytest.fixture
def env(tmp_path):
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    calls = tmp_path / "calls.log"
    for name, body in (("lsblk", FAKE_LSBLK), ("findmnt", FAKE_FINDMNT), ("df", FAKE_DF), ("du", FAKE_DU)):
        (bin_ / name).write_text("#!%s\n%s" % (sys.executable, body))
    for name in DESTRUCTIVE:
        (bin_ / name).write_text('#!/bin/sh\necho "%s $*" >> "%s"\n' % (name, calls))
    for f in bin_.iterdir():
        f.chmod(f.stat().st_mode | stat.S_IXUSR)

    root = tmp_path / "root"
    for d in ("etc", "boot/firmware", "home/pi/fc-v1/robot", "home/pi/fc-v1/run", "home/pi/pupper-fc-runs"):
        (root / d).mkdir(parents=True)
    (root / "etc/hostname").write_text("mts\n")
    (root / "etc/fstab").write_text("PARTUUID=1379fbbd-01  /boot/firmware vfat defaults 0 2\n"
                                    "PARTUUID=1379fbbd-02  /  ext4 defaults,noatime 0 1\n")
    (root / "boot/firmware/cmdline.txt").write_text("console=tty1 root=PARTUUID=1379fbbd-02 rootfstype=ext4 rootwait\n")
    (root / "home/pi/fc-v1/robot/robot_id").write_text("blue\n")
    (root / "home/pi/fc-v1/run/imu_cal.json").write_text("{}")
    (root / "home/pi/pupper-fc-runs/big.bin").write_bytes(b"x" * 5000)

    fixture = tmp_path / "disks.json"
    fixture.write_text(json.dumps(disks()))

    class Env:
        pass
    e = Env()
    e.tmp, e.root, e.fixture, e.calls = tmp_path, root, fixture, calls
    e.set_disks = lambda fx: fixture.write_text(json.dumps(fx))

    def run(*args, stdin=None):
        environ = dict(os.environ, PATH="%s:%s" % (bin_, os.environ["PATH"]), FAKE_DISKS=str(fixture),
                       FC_CLONE_ROOT=str(root))
        return subprocess.run(["bash", str(SCRIPT), *args], capture_output=True, text=True, env=environ,
                              stdin=subprocess.DEVNULL if stdin is None else stdin, timeout=60)
    e.run = run
    yield e
    assert not calls.exists(), "a destructive tool ran: " + calls.read_text()


OK = ["--serial", TARGET_SERIAL, "--hostname", "pupper-red", "--robot-id", "red"]


def refused(r, msg):
    assert r.returncode == 2, (r.returncode, r.stdout, r.stderr)
    assert msg in r.stderr + r.stdout, r.stderr + r.stdout     # real runs tee stderr into the log/stdout


@pytest.mark.parametrize("args, msg", [
    (["--dry-run", "--hostname", "pupper-red", "--robot-id", "red"], "--serial is required"),
    (["--dry-run", "--serial", TARGET_SERIAL, "--robot-id", "red"], "--hostname is required"),
    (["--dry-run", "--serial", TARGET_SERIAL, "--hostname", "pupper-red"], "--robot-id is required"),
    (["--dry-run", "--serial", "a b"] + OK[2:], "bad serial"),
    (OK[:2] + ["--hostname", "Bad_Host", "--robot-id", "red", "--dry-run"], "bad hostname"),
    (OK[:4] + ["--robot-id", "../x", "--dry-run"], "bad robot id"),
    (OK + ["--dry-run", "--mount", "/"], "--mount must be a directory under /mnt"),
    (OK + ["--dry-run", "--exclude", "home/pi/x"], "must be absolute"),
    (OK + ["--bogus"], "unknown argument"),
    (OK + ["--serial"], "missing value"),
])
def test_argument_refusals(env, args, msg):
    refused(env.run(*args), msg)


def test_refuses_own_identity(env):
    refused(env.run("--dry-run", "--serial", TARGET_SERIAL, "--hostname", "mts", "--robot-id", "red"),
            "this Pi's own hostname")
    refused(env.run("--dry-run", "--serial", TARGET_SERIAL, "--hostname", "pupper-red", "--robot-id", "blue"),
            "this Pi's own robot id")


def test_refuses_unknown_serial_and_lists_disks(env):
    r = env.run("--dry-run", "--serial", "NOSUCH123", "--hostname", "pupper-red", "--robot-id", "red")
    refused(r, "no disk has serial NOSUCH123")
    assert "/dev/sda serial=" + BOOT_SERIAL in r.stderr and "/dev/sdb serial=" + TARGET_SERIAL in r.stderr
    assert "loop0" not in r.stderr


def test_refuses_boot_drive(env):
    refused(env.run("--dry-run", "--serial", BOOT_SERIAL, "--hostname", "pupper-red", "--robot-id", "red"),
            "/dev/sda (serial %s) holds / : it is the boot drive" % BOOT_SERIAL)


def test_refuses_disk_holding_boot_firmware(env):
    fx = disks()
    fx["devices"][4]["MOUNTPOINT"] = ""
    fx["findmnt"]["/boot/firmware"] = "/dev/sdb1"            # firmware on sdb, / on sda
    env.set_disks(fx)
    refused(env.run(*OK, "--dry-run"), "holds /boot/firmware : it is the boot drive")


def test_refuses_when_root_disk_unknown(env):
    fx = disks()
    fx["findmnt"] = {}
    env.set_disks(fx)
    refused(env.run(*OK, "--dry-run"), "cannot tell which disk holds /")


@pytest.mark.parametrize("mp, msg", [
    ("/", "mounted as a system filesystem (/)"),
    ("/home", "mounted as a system filesystem (/home)"),
    ("[SWAP]", "mounted as a system filesystem ([SWAP])"),
    ("/media/pi/usb", "mounted at /media/pi/usb: unmount it first"),
])
def test_refuses_mounted_target(env, mp, msg):
    fx = disks()
    fx["devices"][4]["MOUNTPOINT"] = mp
    env.set_disks(fx)
    refused(env.run(*OK, "--dry-run"), msg)


def test_refuses_duplicate_serial(env):
    fx = disks()
    fx["devices"].append({"NAME": "/dev/sdc", "TYPE": "disk", "SERIAL": TARGET_SERIAL, "SIZE": 128 * GB})
    env.set_disks(fx)
    refused(env.run(*OK, "--dry-run"), "several disks have serial")


def test_refuses_small_target(env):
    fx = disks()
    fx["devices"][3]["SIZE"] = 16 * GB
    env.set_disks(fx)
    r = env.run(*OK, "--dry-run")
    refused(r, "/dev/sdb is 16.0 GB; the copy needs about")
    assert "PLAN" in r.stdout                                  # the plan still shows why


def test_refuses_non_stock_layout(env):
    (env.root / "boot/firmware/cmdline.txt").write_text("root=/dev/sda2\n")
    refused(env.run(*OK, "--dry-run"), "no root=PARTUUID")


def test_dry_run_plan(env):
    r = env.run(*OK, "--dry-run")
    assert r.returncode == 0, r.stderr
    out = r.stdout
    assert "mts (robot_id blue) -> pupper-red (robot_id red)" in out
    assert "target     /dev/sdb  serial %s  model 'SanDisk 3.2Gen1'  transport usb  size 128.0 GB" % TARGET_SERIAL in out
    assert "boot drive /dev/sda" in out and "old 1379fbbd" in out
    assert "/dev/sdb1 512 MiB vfat" in out and "/dev/sdb2 rest ext4" in out
    for pat in ("/proc/*", "/home/pi/fc-v1/run/*", "/home/pi/fc-v1/runs/*", "/home/pi/pupper-fc-runs",
                "/home/pi/pupper-foundation-session-20261029", "/var/log/journal/*"):
        assert "--exclude=%s " % pat in out, pat
    assert "/home/pi/pupper-fc-runs  5.0 KB" in out           # du of an existing exclude
    assert "/home/pi/.vscode-server  (absent)" in out
    assert "%s/ /mnt/fc-clone/" % env.root in out
    assert out.rstrip().endswith("DRY RUN: nothing changed.")
    assert not list(env.tmp.glob("*.log*"))                    # dry run writes no log


def test_stale_mount_under_clone_dir_is_allowed(env):
    fx = disks()
    fx["devices"][4]["MOUNTPOINT"] = "/mnt/fc-clone/boot/firmware"
    env.set_disks(fx)
    r = env.run(*OK, "--dry-run")
    assert r.returncode == 0, r.stderr
    assert "stale       /dev/sdb1" in r.stdout


def test_excludes_file_replaces_defaults_but_not_mandatory(env):
    f = env.tmp / "excl.txt"
    f.write_text("# mine\n/home/pi/keep-out   # trailing comment\n\n")
    r = env.run(*OK, "--dry-run", "--excludes-file", str(f), "--exclude", "/opt/big")
    assert r.returncode == 0, r.stderr
    assert "--exclude=/home/pi/keep-out " in r.stdout and "--exclude=/opt/big " in r.stdout
    assert "--exclude=/home/pi/fc-v1/run/* " in r.stdout and "--exclude=/proc/* " in r.stdout
    assert "pupper-fc-runs" not in r.stdout


def test_print_excludes(env):
    r = env.run("--print-excludes")
    assert r.returncode == 0
    lines = r.stdout.split()
    assert "/home/pi/fc-v1/run/*" in lines and "/home/pi/pupper-fc-runs" in lines and "/proc/*" in lines


@pytest.mark.skipif(os.geteuid() == 0, reason="the root check is what stops this test")
def test_real_run_refusal_writes_done_marker(env):
    log = env.tmp / "clone.log"
    r = env.run(*OK, "--yes", "--log", str(log))
    refused(r, "must run as root")
    assert (env.tmp / "clone.log.done").read_text().strip() == "2"
    assert not (env.tmp / "clone.log.pid").exists()
    text = log.read_text()
    assert "PLAN" in text and "must run as root" in text and "FC_CLONE_EXIT 2" in text


def test_real_run_guard_refusal_also_marks_done(env):
    log = env.tmp / "clone.log"
    (env.tmp / "clone.log.done").write_text("0\n")              # stale marker from an earlier run
    r = env.run("--serial", BOOT_SERIAL, "--hostname", "pupper-red", "--robot-id", "red", "--log", str(log))
    refused(r, "it is the boot drive")
    assert (env.tmp / "clone.log.done").read_text().strip() == "2"
