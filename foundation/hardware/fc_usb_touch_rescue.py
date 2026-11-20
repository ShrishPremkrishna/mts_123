#!/usr/bin/env python3
"""Restore an installed Pupper FC touchscreen from its explicitly selected Pi boot partition."""
import argparse
import ast
import errno
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import plistlib
import re
import subprocess
import sys
import tempfile
import time

DIRECTORY = "fc-touch-rescue"
BOOT = Path("/boot/firmware")
HOOK = (b' systemd.unit=kernel-command-line.target'
        b' systemd.run="/usr/bin/python3 -B /boot/firmware/fc-touch-rescue/rescue.py apply"'
        b' systemd.run_success_action=reboot systemd.run_failure_action=none')
MOTOR = ("robot.service", "course-robot.service", "llm-agent.service")
MARKER = "/etc/fc-appliance-boot/appliance-mode"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def atomic(path, data):
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".fc-rescue-", delete=False) as out:
        temp = Path(out.name)
        try:
            out.write(data)
            out.flush()
            os.fsync(out.fileno())
            out.close()
            os.replace(temp, path)
            fd = os.open(path.parent, os.O_RDONLY)
            try:
                try:
                    os.fsync(fd)
                except OSError as exc:
                    if exc.errno not in (errno.EINVAL, errno.ENOTSUP):
                        raise
            finally:
                os.close(fd)
        finally:
            temp.unlink(missing_ok=True)


def write_json(path, value):
    atomic(path, (json.dumps(value, indent=2)+"\n").encode())


def mac_volume(volume, device):
    volume = volume.resolve(strict=True)
    if not re.fullmatch(r"disk[0-9]+s[0-9]+", device):
        raise ValueError("Supply the specific boot partition identifier, for example disk4s1")
    info = plistlib.loads(subprocess.check_output(["/usr/sbin/diskutil","info","-plist",str(volume)]))
    if (info.get("DeviceIdentifier") != device or info.get("Internal") is not False
            or info.get("FilesystemType") != "msdos" or info.get("MountPoint") != str(volume)):
        raise ValueError("Device identifier and mount point must identify the same external mounted FAT partition")
    return volume


def augmented(original):
    body = original.rstrip(b"\r\n")
    if (not body or b"\n" in body or b"\r" in body or b"\x00" in body
            or original[len(body):] not in (b"",b"\n",b"\r\n") or not re.search(rb"(?:^|\s)root=\S+", body)):
        raise ValueError("Expected one Pi kernel command line with a root filesystem")
    if re.search(rb'(?:^|\s)"?(?:systemd\.(?:unit|run(?:_success_action|_failure_action)?)|init|rdinit)=', body):
        raise ValueError("Existing boot override present; inspect cmdline.txt before staging")
    return body + HOOK + original[len(body):]


def touchscreen_selector(data):
    module = ast.parse(data)
    return any(isinstance(node, ast.Assign) and any(isinstance(k, ast.Name) and k.id == "APPLIANCE_TARGET" for k in node.targets)
               and isinstance(node.value, ast.Constant) and node.value.value == "graphical.target" for node in module.body)


def check_firmware_config(volume):
    pending = [volume/"config.txt"]
    seen = set()
    while pending:
        file = pending.pop().resolve(strict=True)
        if file in seen:
            continue
        if not file.is_relative_to(volume):
            raise ValueError("Firmware config include escapes the selected boot partition")
        seen.add(file)
        for row in file.read_text().splitlines():
            row = row.split("#",1)[0].strip()
            if row.startswith("include "):
                pending.append(volume/row[8:].strip())
            match = re.fullmatch(r"(cmdline|os_prefix)\s*=\s*(.*)",row)
            if match and match.group(2).strip() != ("cmdline.txt" if match.group(1)=="cmdline" else ""):
                raise ValueError("This Pi selects an alternate kernel command line; inspect its firmware config")


def stage(volume, selector):
    volume = volume.resolve(strict=True)
    check_firmware_config(volume)
    cmd = volume/"cmdline.txt"
    original = cmd.read_bytes()
    armed = augmented(original)
    payload = selector.read_bytes()
    if not touchscreen_selector(payload):
        raise ValueError("Select the new graphical FC appliance selector; the earlier one selects headless boot")
    folder = volume/DIRECTORY
    if folder.exists():
        raise ValueError("A rescue folder already exists. Inspect its receipt or cancel the staged rescue first.")
    rescue = Path(__file__).resolve().read_bytes()
    with tempfile.TemporaryDirectory(dir=volume, prefix=".fc-rescue-stage-") as tmp:
        work = Path(tmp)/DIRECTORY
        work.mkdir()
        files = {"cmdline.original":original, "rescue.py":rescue, "fc_appliance.py":payload}
        for name,data in files.items():
            atomic(work/name, data)
        write_json(work/"manifest.json", dict(files={k:sha(v) for k,v in files.items()}, armed=sha(armed)))
        work.rename(folder)
    if cmd.read_bytes() != original:
        raise ValueError("cmdline.txt changed during staging; no hook installed")
    atomic(cmd, armed)
    if cmd.read_bytes() != armed:
        raise OSError("Unable to verify the written boot command line")
    return "Staged; eject this boot device before removing it."


def inspect(volume):
    folder = volume/DIRECTORY
    ledger = json.loads((folder/"manifest.json").read_text())
    names = ("cmdline.original", "rescue.py", "fc_appliance.py")
    if set(ledger["files"]) != set(names):
        raise ValueError("Unexpected rescue manifest")
    if any(sha((folder/k).read_bytes()) != ledger["files"][k] for k in names):
        raise ValueError("Rescue payload or saved original differs from the staged manifest")
    original = (folder/"cmdline.original").read_bytes()
    armed = augmented(original)
    if sha(armed) != ledger["armed"]:
        raise ValueError("Unexpected staged hook")
    current = (volume/"cmdline.txt").read_bytes()
    if current not in (original, armed):
        raise ValueError("cmdline.txt changed separately; preserve it and inspect the saved original")
    return folder, original, current == armed


def cancel(volume):
    folder, original, armed = inspect(volume)
    if armed:
        atomic(volume/"cmdline.txt", original)
    allowed = {"cmdline.original","rescue.py","fc_appliance.py","manifest.json","result.json"}
    # macOS puts its AppleDouble metadata next to files on FAT.
    companions = {p.name for p in folder.iterdir() if p.name.startswith("._") and p.name[2:] in allowed
                  and p.is_file() and p.read_bytes()[:4] == b"\x00\x05\x16\x07"}
    if {p.name for p in folder.iterdir()} - allowed - companions:
        raise ValueError("Hook cleared; unexpected files in rescue folder. Folder preserved.")
    if (folder/"result.json").exists():
        return "Hook cleared; repair receipt retained on boot partition."
    for name in allowed | companions:
        (folder/name).unlink(missing_ok=True)
    folder.rmdir()
    return "Original boot command line restored."


def migrate(folder, root):
    spec = importlib.util.spec_from_file_location("fc_rescue_selector", folder/"fc_appliance.py")
    selector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(selector)
    app = selector.FcAppliance(str(root))
    app.enable()
    report = app.inspect(audit=False)
    expected = {
        "kernel command line in /boot/firmware/cmdline.txt overrides the boot target (systemd.unit=kernel-command-line.target)",
        "systemctl --root reports default target kernel-command-line.target, files say graphical.target",
    }
    # Some images alias /boot/cmdline.txt to the same active firmware file.
    if (root/"boot/cmdline.txt").resolve() == (root/"boot/firmware/cmdline.txt").resolve():
        expected.add("kernel command line in /boot/cmdline.txt overrides the boot target (systemd.unit=kernel-command-line.target)")
    extra = set(report["problems"])-expected
    if report.get("phase") != "enabled" or report["boot_target"].get("effective")!="graphical.target" or extra:
        raise ValueError("Selector could not confirm a protected graphical boot: "+", ".join(sorted(extra)))


def repair(volume, system_root, run, *, wait_s=65, stable_s=7, migrate_selector=migrate):
    folder, original, armed = inspect(volume)
    if not armed:
        raise ValueError("Rescue hook is not armed")
    marker = system_root/MARKER.lstrip("/")
    target = system_root/"etc/systemd/system/default.target"
    receipt = {"success":False, "cmdline_restored":False}
    try:
        if not marker.is_file() or not (marker.parent/"state.json").is_file():
            raise ValueError("Expected the existing FC appliance marker and boot journal")
        migrate_selector(folder, system_root)
        if not target.is_symlink() or Path(os.readlink(target)).name != "graphical.target" or not marker.is_file():
            raise ValueError("Graphical boot and appliance marker not confirmed")
        directory = system_root/"etc/systemd/system"
        condition = "ConditionPathExists=!"+MARKER
        for name in MOTOR:
            dropin = directory/(name+".d")/"90-fc-appliance.conf"
            if condition not in dropin.read_text().splitlines():
                raise ValueError("Motion suppression missing for "+name)
        # The root journal, graphical target and motion fences must reach disk before disarming the FAT hook.
        os.sync()
        if not inspect(volume)[2]:
            raise ValueError("Boot command line changed during touchscreen repair")
        atomic(volume/"cmdline.txt",original)
        receipt["cmdline_restored"] = True
        run(["/usr/bin/systemctl","daemon-reload"],check=True,timeout=35)
        run(["/usr/bin/systemctl","--no-block","start","graphical.target","NetworkManager.service","ssh.service"],
            check=True,timeout=15)
        deadline = time.monotonic()+wait_s
        units = ("display-manager.service","pupper-gui.service","NetworkManager.service","ssh.service",*MOTOR)
        previous_gui, stable_since = None, 0.
        while True:
            states = run(["/usr/bin/systemctl","is-active",*units],check=False,capture_output=True,text=True,timeout=8).stdout.splitlines()
            if len(states) != len(units):
                raise ValueError("Cannot read all selected systemd service states")
            receipt["services"] = dict(zip(units,states))
            if any(s in ("active","activating","reloading") for s in states[4:]):
                raise ValueError("A legacy robot service is running; inspect before any further boot operation")
            displays = [str(x.relative_to(system_root)) for x in
                        (system_root/"run/user/1000/wayland-0",system_root/"tmp/.X11-unix/X0") if x.is_socket()]
            receipt["display_sockets"] = displays
            if all(s=="active" for s in states[:4]) and displays:
                props = run(["/usr/bin/systemctl","show","pupper-gui.service","-p","MainPID","-p","NRestarts"],
                            check=True,capture_output=True,text=True,timeout=8).stdout
                gui = dict(line.split("=",1) for line in props.splitlines() if "=" in line)
                if gui != previous_gui:
                    previous_gui, stable_since = gui, time.monotonic()
                if (gui.get("MainPID","").isdigit() and int(gui["MainPID"])>0 and gui.get("NRestarts","").isdigit()
                        and time.monotonic()-stable_since >= stable_s):
                    receipt["gui"] = dict(gui, stable_s=round(time.monotonic()-stable_since,2))
                    break
            else:
                previous_gui = None
            if time.monotonic() >= deadline:
                raise ValueError("Touchscreen or SSH service is not active; see service states in the receipt")
            time.sleep(1)
        receipt["success"] = True
    except Exception as error:
        receipt["error"] = str(error)
        # On failure retain the diagnostic and keep SSH/NM reachable if those units are available.
        run(["/usr/bin/systemctl","--no-block","start","NetworkManager.service","ssh.service"], check=False,timeout=15)
        raise
    finally:
        write_json(folder/"result.json",receipt)
        os.sync()
    return "Touchscreen stack and SSH active. Default graphical boot restored."


def main():
    if sys.argv[1:] == ["apply"]:
        if (sys.platform!="linux" or os.geteuid()!=0 or Path(__file__).resolve()!=BOOT/DIRECTORY/"rescue.py"
                or not BOOT.is_mount()):
            raise ValueError("Apply runs only from the Pi firmware mount as the one-shot boot service")
        print(repair(BOOT,Path("/"),subprocess.run))
        return
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("action",choices=("stage","cancel","status"))
    p.add_argument("volume",type=Path)
    p.add_argument("--device",required=True,help="external FAT partition identifier from diskutil list external physical")
    args=p.parse_args()
    vol=mac_volume(args.volume,args.device)
    if args.action=="stage":
        print(stage(vol,Path(__file__).with_name("fc_appliance.py")))
    elif args.action=="cancel":
        print(cancel(vol))
    else:
        folder,_,armed=inspect(vol)
        print("Hook "+("armed" if armed else "cleared"))
        if (folder/"result.json").exists():
            print((folder/"result.json").read_text())


if __name__=="__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        sys.exit("Touchscreen rescue: "+str(error))
