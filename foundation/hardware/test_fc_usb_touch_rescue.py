from pathlib import Path
from types import SimpleNamespace
import json
import subprocess
import socket

import pytest

from foundation.hardware import fc_usb_touch_rescue as rescue

SELECTOR = Path(__file__).with_name("fc_appliance.py")
LINE = b"console=tty1 root=PARTUUID=12345678-02 rootwait quiet fsck.repair=yes\r\n"


@pytest.fixture
def volumes(tmp_path):
    boot = tmp_path/"root/boot/firmware"
    boot.mkdir(parents=True)
    (boot/"cmdline.txt").write_bytes(LINE)
    (boot/"config.txt").write_text("[all]\ndtparam=audio=on\n")
    return boot, tmp_path/"root"


def test_stage_and_cancel_round_trip_and_no_unrelated_changes(volumes):
    boot,_ = volumes
    neighbor=boot/"unrelated.tar.gz"
    neighbor.write_bytes(b"unrelated")
    rescue.stage(boot,SELECTOR)
    assert rescue.inspect(boot)[2]
    assert (boot/"cmdline.txt").read_bytes()==LINE[:-2]+rescue.HOOK+b"\r\n"
    with pytest.raises(ValueError):
        rescue.stage(boot,SELECTOR)
    rescue.cancel(boot)
    assert (boot/"cmdline.txt").read_bytes()==LINE
    assert neighbor.read_bytes()==b"unrelated" and not (boot/rescue.DIRECTORY).exists()


@pytest.mark.parametrize("bad",[b"root=x\nsecond=cmdline\n", b"root=y systemd.unit=rescue.target\n",
                                 b"root=y systemd.run=/other", b"root=x init=/bin/sh",b"quiet\n"])
def test_refuse_conflicting_cmdline(volumes,bad):
    boot,_ = volumes
    (boot/"cmdline.txt").write_bytes(bad)
    with pytest.raises(ValueError):
        rescue.stage(boot,SELECTOR)
    assert (boot/"cmdline.txt").read_bytes()==bad and not (boot/rescue.DIRECTORY).exists()


def test_refuse_wrong_selector_alternate_config_modified_cmdline_or_payload(volumes,tmp_path):
    boot,_ = volumes
    old=tmp_path/"old.py"
    old.write_text('HEADLESS_TARGET="multi-user.target"')
    with pytest.raises(ValueError,match="headless"):
        rescue.stage(boot,old)
    (boot/"config.txt").write_text("include aux.txt")
    (boot/"aux.txt").write_text("cmdline=alternate.txt")
    with pytest.raises(ValueError,match="alternate"):
        rescue.stage(boot,SELECTOR)
    (boot/"aux.txt").write_text("")
    rescue.stage(boot,SELECTOR)
    armed=(boot/"cmdline.txt").read_bytes()
    (boot/"cmdline.txt").write_bytes(armed+b" x=1")
    with pytest.raises(ValueError,match="separately"):
        rescue.cancel(boot)
    (boot/"cmdline.txt").write_bytes(armed)
    (boot/rescue.DIRECTORY/"fc_appliance.py").write_bytes(b"changed")
    with pytest.raises(ValueError,match="payload"):
        rescue.cancel(boot)


@pytest.mark.parametrize("fault",[None, "selector", "condition", "ssh", "motor"])
def test_repair_requires_motion_fence_and_observed_gui_ssh(volumes,fault):
    boot,root=volumes
    rescue.stage(boot,SELECTOR)
    marker=root/rescue.MARKER.lstrip("/")
    marker.parent.mkdir(parents=True)
    marker.write_text("present")
    (marker.parent/"state.json").write_text("{}")
    display=root/"tmp/.X11-unix/X0"
    display.parent.mkdir(parents=True)
    # Shorten the bind path for Darwin's 104-byte Unix socket limit.
    import os
    old=os.getcwd()
    try:
        os.chdir(display.parent)
        with socket.socket(socket.AF_UNIX) as srv: srv.bind("X0")
    finally:
        os.chdir(old)
    etc=root/"etc/systemd/system"
    etc.mkdir(parents=True)
    calls=[]
    def migrate(folder,root):
        assert rescue.inspect(boot)[2]
        if fault=="selector":raise subprocess.CalledProcessError(1,"selector")
        (etc/"default.target").symlink_to("/usr/lib/systemd/system/graphical.target")
        for name in rescue.MOTOR:
            d=etc/(name+".d");d.mkdir()
            (d/"90-fc-appliance.conf").write_text("wrong" if fault=="condition" else "ConditionPathExists=!"+rescue.MARKER)
    def run(cmd,**kw):
        calls.append(cmd)
        if "show" in cmd:
            return SimpleNamespace(stdout="NRestarts=0\nMainPID=1\n")
        if "is-active" in cmd:
            services=["active"]*4+["inactive"]*3
            if fault=="ssh":services[3]="inactive"
            if fault=="motor":services[4]="active"
            return SimpleNamespace(stdout="\n".join(services))
        return SimpleNamespace(stdout="")
    if fault:
        with pytest.raises((ValueError,subprocess.CalledProcessError)):
            rescue.repair(boot,root,run,wait_s=0,stable_s=0,migrate_selector=migrate)
    else:
        rescue.repair(boot,root,run,wait_s=0,stable_s=0,migrate_selector=migrate)
    result=json.loads((boot/rescue.DIRECTORY/"result.json").read_text())
    assert result["success"]==(fault is None)
    assert result["cmdline_restored"]==(fault not in ("selector","condition"))
    assert rescue.inspect(boot)[2]==(fault in ("selector","condition")) and marker.is_file()
    assert all(not any(x in rescue.MOTOR for x in cmd) for cmd in calls if "start" in cmd)
    assert not any("graphical.target" in cmd for cmd in calls) if fault in ("condition","selector") else True


def test_interrupted_root_migration_keeps_the_hook_and_saved_boot_bytes(volumes):
    boot,root=volumes
    rescue.stage(boot,SELECTOR)
    folder=boot/rescue.DIRECTORY
    marker=root/rescue.MARKER.lstrip("/")
    marker.parent.mkdir(parents=True)
    marker.write_text("existing FC motor interlock")
    (marker.parent/"state.json").write_text("{}")
    def power_loss(folder,root):
        assert rescue.inspect(boot)[2]
        raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        rescue.repair(boot,root,lambda *_a,**_k:pytest.fail("no service action before durable migration"),
                      migrate_selector=power_loss)
    assert rescue.inspect(boot)[2] and (folder/"cmdline.original").read_bytes()==LINE
    assert marker.read_text()=="existing FC motor interlock"
