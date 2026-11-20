#!/usr/bin/env python3
"""Boot personality switch for the FC robot-soccer appliance (Raspberry Pi, systemd).

    fc_appliance.py enable    next boot comes up headless with course/vendor robot units skipped
    fc_appliance.py normal    undo: restore the recorded boot target and remove everything owned
    fc_appliance.py status    current mode, problems, and remaining boot launch surfaces

What `enable` writes (nothing else is touched, all of it is removed by `normal`):

  /etc/fc-appliance-boot/state.json         journal: original boot target and owned drop-ins
  /etc/fc-appliance-boot/appliance-mode     marker; the single switch the drop-ins key on
  /etc/systemd/system/<unit>.d/90-fc-appliance.conf
                                            [Unit] ConditionPathExists=!<marker>, one per
                                            suppressed unit (additive, unit files untouched)
  /etc/systemd/system/default.target        -> multi-user.target (the original link is journaled)

What it never does: enable/disable/mask units, start or stop anything, reload systemd, signal
processes, or touch network, SSH, mDNS, Bluetooth or battery configuration. Changes take effect
at the next boot. `--root DIR` operates on an offline OS tree the way `systemctl --root` does and
never touches the live system; only `enable`/`normal` on the live root need root.

Exit codes: 0 ok, 1 failed (see message), 2 usage, 3 applied/inspected but degraded, 4 not root.
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import fnmatch
import json
import os
import re
import shlex
import signal
import stat
import subprocess
import sys
import time
import uuid
from collections import deque

TOOL = "fc_appliance"
SCHEMA = 1

EXIT_OK, EXIT_FAIL, EXIT_USAGE, EXIT_DEGRADED, EXIT_PERM = 0, 1, 2, 3, 4

OWNED_DIR = "/etc/fc-appliance-boot"
MARKER = OWNED_DIR + "/appliance-mode"
STATE = OWNED_DIR + "/state.json"
ETC_UNITS = "/etc/systemd/system"
DEFAULT_LINK = ETC_UNITS + "/default.target"
DROPIN_NAME = "90-fc-appliance.conf"
TMP_PREFIX = ".fc-appliance-tmp."
HEADLESS_TARGET = "multi-user.target"

# Highest priority first, as systemd searches them.
UNIT_DIRS = (
    "/etc/systemd/system.control", "/run/systemd/system.control",
    "/run/systemd/transient", "/run/systemd/generator.early",
    ETC_UNITS, "/etc/systemd/system.attached",
    "/run/systemd/system", "/run/systemd/system.attached", "/run/systemd/generator",
    "/usr/local/lib/systemd/system",
    "/lib/systemd/system",
    "/usr/lib/systemd/system", "/run/systemd/generator.late",
)
PACKAGE_UNIT_DIRS = ("/lib/systemd/system/", "/usr/lib/systemd/system/")

# Course/vendor units that drive the robot and conflict with the appliance workload. They are
# suppressed whether or not they are installed yet, so a later course setup run stays covered.
CONFLICTING_UNITS = (
    "robot.service",
    "course-robot.service",
    "pupper-gui.service",
    "llm-agent.service",
    # Enabled on the archived course image; these serve the desktop, printer,
    # audio UI or legacy local robot console. Socket/path activation is included.
    "cups.path", "cups.service", "cups.socket", "cups-browsed.service",
    "change_tty1_permissions.service", "glamor-test.service", "rpi-display-backlight.service",
    "volume-max.service", "wayvnc-control.service", "wayvnc.service",
)
DISPLAY_MANAGER_ALIAS = "display-manager.service"
# lightdm is what the recorded image aliases display-manager.service to; the rest only if installed.
DISPLAY_MANAGERS = ("lightdm.service",)
OTHER_DISPLAY_MANAGERS = (
    "gdm.service", "gdm3.service", "sddm.service", "greetd.service",
    "lxdm.service", "xdm.service", "nodm.service", "slim.service",
)

# Never suppressed (refused for --also-suppress); also the reason shown for kept units.
PROTECTED = (
    ("ssh*", "SSH"),
    ("NetworkManager*", "network"), ("wpa_supplicant*", "network"), ("dhcpcd*", "network"),
    ("networking.service", "network"), ("ModemManager*", "network"), ("comitup*", "network"),
    ("tailscaled*", "network"),
    ("avahi-daemon*", "mDNS"),
    ("bluetooth*", "Bluetooth"), ("hciuart*", "Bluetooth"), ("bthelper@*", "Bluetooth"),
    ("battery_monitor*", "battery"),
    ("dbus*", "OS"), ("systemd-*", "OS"), ("polkit*", "OS"),
    ("getty@*", "console"), ("serial-getty@*", "console"),
)
SUPERVISORS = ("docker.service", "containerd.service", "podman-restart.service",
               "supervisor.service", "pm2-*.service")

SUPPRESSIBLE_RE = re.compile(r"^[A-Za-z0-9:_.\\-]+(@[A-Za-z0-9:_.\\-]*)?\.(service|socket|timer|path)$")
USER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]{0,31}\$?$")
SAFE_TOKEN_RE = re.compile(r"^[A-Za-z0-9_./@%+~:-]{1,160}$")
CONDITION_RESET_RE = re.compile(r"Condition[A-Za-z]+\s*=\s*")
ENV_LINE_RE = re.compile(r"^(source\s|\.\s|export\s|alias\s|unset\s|[A-Za-z_][A-Za-z0-9_]*=)")
HINT_RE = re.compile(
    r"(?i)(ros2?\s+(launch|run)|roslaunch|rosrun|\.launch\.py|robot|pupper|motor|llm|homing|"
    r"teleop|neural|startx|xinit|\blabwc\b|\bwayfire\b)"
)
SYSTEM_EXEC_PREFIXES = ("/usr/bin/", "/usr/sbin/", "/bin/", "/sbin/", "/usr/lib/", "/lib/",
                        "/usr/libexec/", "/usr/share/")
NONSYSTEM_ARG_PREFIXES = ("/home/", "/opt/", "/usr/local/", "/srv/", "/root/", "/mnt/", "/media/",
                          "/tmp/", "/var/tmp/", "~", "./", "$HOME", "${HOME}")
SHELLS = ("sh", "bash", "dash", "zsh")

SYSTEMCTL_CANDIDATES = ("/usr/bin/systemctl", "/bin/systemctl")
SYSTEMCTL_ENV = {"PATH": "/usr/sbin:/usr/bin:/sbin:/bin", "LC_ALL": "C",
                 "SYSTEMD_PAGER": "", "SYSTEMD_COLORS": "0"}

DROPIN_TEXT = (
    "# Managed by fc_appliance.py; removed by `fc_appliance.py normal`. Do not edit.\n"
    "# Skips this unit while the FC appliance boot personality is enabled.\n"
    "[Unit]\n"
    "ConditionPathExists=!" + MARKER + "\n"
).encode()
MARKER_TEXT = (
    b"FC appliance boot personality is enabled (managed by fc_appliance.py).\n"
    b"Removing this file re-activates the units carrying 90-fc-appliance.conf drop-ins.\n"
)

NOT_INSPECTED = (
    "running system (processes, loaded units, sessions): never queried or changed",
    "udev RUN+= rules, systemd generators, SysV init scripts, NetworkManager dispatcher scripts, at jobs",
    "workloads started by container/process supervisors (docker restart policies, supervisor, pm2)",
    "what scripts and executables do once started (only launch points are inventoried)",
)


class ApplianceError(Exception):
    """Operational refusal or failure; the message is shown to the operator."""


class PrivilegeError(ApplianceError):
    pass


class StateError(ApplianceError):
    """The journal exists but cannot be trusted."""


def _strerror(exc):
    return exc.strerror or str(exc)


class RootFS:
    """Read access to an OS tree by in-image absolute path, never escaping `root`."""

    READ_LIMIT = 256 * 1024

    def __init__(self, root):
        self.root = os.path.realpath(root)
        self.live = self.root == "/"

    def host(self, path):
        if not path.startswith("/") or ".." in path.split("/"):
            raise ValueError("not a normalized absolute path: %r" % path)
        return self.root.rstrip("/") + path if path != "/" else self.root

    def chase(self, path, follow_final=True):
        """Resolve symlinks relative to the root, like chroot would.

        Returns (resolved_path, exists). exists is None when a component could not be examined
        (permission denied), so callers never mistake "unreadable" for "absent"; resolved_path
        is None on a symlink loop.
        """
        parts = deque(p for p in path.split("/") if p and p != ".")
        resolved = []
        hops = 0
        while parts:
            comp = parts.popleft()
            if comp == "..":
                if resolved:
                    resolved.pop()
                continue
            cand = resolved + [comp]
            hostpath = self.host("/" + "/".join(cand))
            try:
                st = os.lstat(hostpath)
            except OSError as exc:
                rest = [p for p in parts if p != ".."]
                missing = exc.errno in (errno.ENOENT, errno.ENOTDIR)
                return "/" + "/".join(cand + rest), (False if missing else None)
            if stat.S_ISLNK(st.st_mode) and (parts or follow_final):
                hops += 1
                if hops > 40:
                    return None, False
                try:
                    target = os.readlink(hostpath)
                except OSError:
                    return "/" + "/".join(cand), False
                if target.startswith("/"):
                    resolved = []
                parts.extendleft(reversed([p for p in target.split("/") if p and p != "."]))
                continue
            resolved = cand
        return "/" + "/".join(resolved), True

    def _lhost(self, path):
        """Host path of `path` with every parent symlink resolved inside the root."""
        parent, exists = self.chase(os.path.dirname(path))
        if parent is None or not exists:
            return None
        return self.host(parent.rstrip("/") + "/" + os.path.basename(path))

    def kind(self, path):
        """absent | file | dir | symlink | other, without following the final component."""
        hostpath = self._lhost(path)
        if hostpath is None:
            return "absent"
        try:
            mode = os.lstat(hostpath).st_mode
        except OSError:
            return "absent"
        if stat.S_ISLNK(mode):
            return "symlink"
        if stat.S_ISDIR(mode):
            return "dir"
        if stat.S_ISREG(mode):
            return "file"
        return "other"

    def readlink(self, path):
        hostpath = self._lhost(path)
        try:
            return os.readlink(hostpath) if hostpath else None
        except OSError:
            return None

    def read_bytes(self, path, limit=None):
        """Returns (data, status); status is ok | absent | unreadable | not-a-file."""
        resolved, exists = self.chase(path)
        if resolved is None or exists is None:
            return None, "unreadable"
        if not exists:
            return None, "absent"
        hostpath = self.host(resolved)
        try:
            if not stat.S_ISREG(os.lstat(hostpath).st_mode):
                return None, "not-a-file"
            fd = os.open(hostpath, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC)
            with os.fdopen(fd, "rb") as fh:
                return fh.read(limit or self.READ_LIMIT), "ok"
        except FileNotFoundError:
            return None, "absent"
        except OSError:
            return None, "unreadable"

    def read_text(self, path):
        data, status = self.read_bytes(path)
        return (data.decode("utf-8", "replace") if data is not None else None), status

    def listdir(self, path):
        """Returns (sorted names, status); status is ok | absent | unreadable | not-a-dir."""
        resolved, exists = self.chase(path)
        if resolved is None or exists is None:
            return None, "unreadable"
        if not exists:
            return None, "absent"
        try:
            return sorted(os.listdir(self.host(resolved)))[:2000], "ok"
        except NotADirectoryError:
            return None, "not-a-dir"
        except FileNotFoundError:
            return None, "absent"
        except OSError:
            return None, "unreadable"

    def executable(self, path):
        resolved, exists = self.chase(path)
        if not exists:
            return False
        try:
            return bool(os.lstat(self.host(resolved)).st_mode & 0o111)
        except OSError:
            return False


def parse_unit(text, into=None):
    """Minimal unit-file parser: {section: {key: [values]}}; an empty assignment resets the key."""
    data = {} if into is None else into
    section = None
    pending = ""
    for raw in text.splitlines():
        line = pending + raw.strip()
        if line.endswith("\\"):
            pending = line[:-1].rstrip() + " "
            continue
        pending = ""
        if not line or line[0] in "#;":
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1]
            continue
        if section is None or "=" not in line:
            continue
        key, value = (part.strip() for part in line.split("=", 1))
        if value:
            data.setdefault(section, {}).setdefault(key, []).append(value)
        else:
            data.setdefault(section, {})[key] = []
    return data


def command_head(cmdline):
    """Describe a command without leaking it: program, optional script path, review reasons.

    Arguments, environment assignments and inline shell text are never included in the result.
    """
    toks = None
    try:
        toks = shlex.split(cmdline.strip().lstrip("-@:+!"))
    except ValueError:
        pass
    if not toks:
        toks = cmdline.strip().lstrip("-@:+!").split()
    if not toks:
        return {"head": "", "reasons": []}
    prog = toks[0]
    reasons = []
    if prog.startswith(("~", "./", "$HOME", "${HOME}")) or (
            prog.startswith("/") and not prog.startswith(SYSTEM_EXEC_PREFIXES)):
        reasons.append("non-system executable")
    shown = [prog if SAFE_TOKEN_RE.match(prog) else "<redacted>"]
    script = None
    for tok in toks[1:]:
        candidates = [tok] + ([tok.split("=", 1)[1]] if "=" in tok else [])
        if any(c.startswith(NONSYSTEM_ARG_PREFIXES) for c in candidates):
            if "non-system executable" not in reasons:
                reasons.append("non-system code path")
            if script is None and tok.startswith(("/", "~", "./")) and SAFE_TOKEN_RE.match(tok):
                script = tok
    if script:
        shown.append(script)
    if os.path.basename(prog) in SHELLS and any(re.match(r"^-[a-z]*c$", t) for t in toks[1:]):
        reasons.append("inline shell command")
    if HINT_RE.search(cmdline):
        reasons.append("robot-workload hint")
    return {"head": " ".join(shown), "reasons": reasons}


def scan_lines(text, skip_env=False):
    """Count active script lines and collect line numbers matching robot-workload hints."""
    active, flagged = 0, []
    for number, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if skip_env and ENV_LINE_RE.match(stripped):
            continue
        active += 1
        if HINT_RE.search(stripped) and len(flagged) < 8:
            flagged.append(number)
    return active, flagged


def protected_reason(unit):
    for pattern, reason in PROTECTED:
        if fnmatch.fnmatchcase(unit, pattern):
            return reason
    return None


def _now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class FcAppliance:
    def __init__(self, root="/", dry_run=False, systemctl="auto"):
        self.fs = RootFS(root)
        self.dry_run = dry_run
        self.systemctl = systemctl
        self.actions = []
        self._enabled_cache = None
        self._aliases = None

    # ---- small helpers -------------------------------------------------------------------

    @staticmethod
    def dropin_dir(unit):
        return "%s/%s.d" % (ETC_UNITS, unit)

    @classmethod
    def dropin_path(cls, unit):
        return "%s/%s" % (cls.dropin_dir(unit), DROPIN_NAME)

    def _check_privilege(self):
        if self.fs.live and not self.dry_run and os.geteuid() != 0:
            raise PrivilegeError(
                "enable/normal change the live system and must run as root (use sudo); "
                "status, --dry-run and --root DIR do not need root")

    def _require_real_dirs(self, path):
        """Every component of `path` must be a real directory, so writes cannot leave the root."""
        prefix = ""
        for comp in path.strip("/").split("/"):
            prefix += "/" + comp
            kind = self.fs.kind(prefix)
            if kind != "dir":
                raise ApplianceError(
                    "%s is %s under %s, expected a real directory; refusing to write there"
                    % (prefix, "missing" if kind == "absent" else "a " + kind, self.fs.root))

    def _locked(self, fn):
        """Run fn while holding an exclusive lock on the unit directory (no lock file left behind)."""
        try:
            fd = os.open(self.fs.host(ETC_UNITS), os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        except OSError:
            return fn()  # the preflight reports a missing or unusable unit directory
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                raise ApplianceError("another fc_appliance run holds the lock on " + ETC_UNITS)
            return fn()
        finally:
            os.close(fd)

    # ---- mutators: the only code that writes; all honour --dry-run -----------------------

    def _sync_dir(self, path):
        try:
            fd = os.open(self.fs.host(path), os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        except OSError:
            return
        try:
            os.fsync(fd)
        except OSError:
            pass
        finally:
            os.close(fd)

    def _mkdir(self, path):
        self.actions.append("mkdir " + path)
        if self.dry_run:
            return
        hostpath = self.fs.host(path)
        os.mkdir(hostpath, 0o755)
        os.chmod(hostpath, 0o755)
        self._sync_dir(os.path.dirname(path))

    def _write_fd(self, hostpath, data):
        fd = os.open(hostpath, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                     0o644)
        try:
            with os.fdopen(fd, "wb") as fh:
                os.fchmod(fh.fileno(), 0o644)
                fh.write(data)
                fh.flush()
                os.fsync(fh.fileno())
        except BaseException:
            try:
                os.unlink(hostpath)
            except OSError:
                pass
            raise

    def _write_new(self, path, data):
        """Create a file that must not exist yet (never clobbers, never follows a symlink)."""
        self.actions.append("create " + path)
        if self.dry_run:
            return
        self._write_fd(self.fs.host(path), data)
        self._sync_dir(os.path.dirname(path))

    def _tmp_host(self, path):
        # Atomic replacement must never unlink a pre-existing administrator file.
        return self.fs.host(os.path.dirname(path) + "/" + TMP_PREFIX + uuid.uuid4().hex)

    def _write_atomic(self, path, data, note="write"):
        self.actions.append("%s %s" % (note, path))
        if self.dry_run:
            return
        tmp = self._tmp_host(path)
        self._write_fd(tmp, data)
        try:
            os.replace(tmp, self.fs.host(path))
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        self._sync_dir(os.path.dirname(path))

    def _symlink_atomic(self, path, target):
        self.actions.append("symlink %s -> %s" % (path, target))
        if self.dry_run:
            return
        tmp = self._tmp_host(path)
        os.symlink(target, tmp)
        try:
            os.replace(tmp, self.fs.host(path))
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        self._sync_dir(os.path.dirname(path))

    def _unlink(self, path):
        self.actions.append("remove " + path)
        if self.dry_run:
            return
        os.unlink(self.fs.host(path))
        self._sync_dir(os.path.dirname(path))

    def _rmdir(self, path, tolerate_nonempty=True):
        """Remove an empty directory; returns False when it was left because it is not empty."""
        if self.dry_run:
            self.actions.append("rmdir " + path + " (if empty)")
            return True
        try:
            os.rmdir(self.fs.host(path))
        except FileNotFoundError:
            return True
        except OSError as exc:
            if tolerate_nonempty and exc.errno in (errno.ENOTEMPTY, errno.EEXIST):
                return False
            raise
        self.actions.append("rmdir " + path)
        self._sync_dir(os.path.dirname(path))
        return True

    # ---- journal -------------------------------------------------------------------------

    def _load_state(self):
        kind = self.fs.kind(STATE)
        if kind == "absent":
            return None
        if kind != "file":
            raise StateError("%s is a %s, not a regular file" % (STATE, kind))
        text, status = self.fs.read_text(STATE)
        if text is None:
            raise StateError("%s is %s" % (STATE, status))
        try:
            state = json.loads(text)
            if not isinstance(state, dict) or state.get("tool") != TOOL:
                raise ValueError("not an fc_appliance journal")
            if state.get("schema") != SCHEMA:
                raise ValueError("unsupported schema %r" % (state.get("schema"),))
            if state.get("phase") not in ("enabling", "enabled", "restoring"):
                raise ValueError("unknown phase")
            default = state["default_target"]
            original = default["original"]
            if original["kind"] not in ("absent", "symlink"):
                raise ValueError("bad original boot target")
            if original["kind"] == "symlink" and not isinstance(original.get("target"), str):
                raise ValueError("bad original boot target")
            if default.get("applied") is not None and not isinstance(default["applied"], str):
                raise ValueError("bad applied boot target")
            for entry in state["dropins"]:
                if not SUPPRESSIBLE_RE.match(entry["unit"]):
                    raise ValueError("bad unit name in journal")
                entry["dir_created"] = bool(entry.get("dir_created"))
                entry["adopted"] = bool(entry.get("adopted"))
        except (ValueError, KeyError, TypeError) as exc:
            raise StateError("%s is not usable: %s" % (STATE, exc))
        return state

    def _save_state(self, state):
        state["updated_utc"] = _now()
        data = (json.dumps(state, indent=2, sort_keys=True) + "\n").encode()
        self._write_atomic(STATE, data, note="journal phase=%s" % state["phase"])

    # ---- unit and boot-target lookup -----------------------------------------------------

    def _enabled_map(self):
        """{unit: [target, ...]} from the .wants/.requires/.upholds links under /etc."""
        if self._enabled_cache is None:
            enabled = {}
            names, _ = self.fs.listdir(ETC_UNITS)
            for name in names or ():
                stem, dot, suffix = name.rpartition(".")
                if not dot or suffix not in ("wants", "requires", "upholds"):
                    continue
                entries, _ = self.fs.listdir(ETC_UNITS + "/" + name)
                for entry in entries or ():
                    enabled.setdefault(entry, []).append(stem)
            self._enabled_cache = enabled
        return self._enabled_cache

    def _unit_info(self, unit):
        """Where a unit comes from in this tree: state, path, origin, alias target."""
        candidates = [unit]
        prefix, at, rest = unit.partition("@")
        if at and not rest.startswith("."):
            candidates.append("%s@.%s" % (prefix, rest.rpartition(".")[2]))
        for cand in candidates:
            for directory in UNIT_DIRS:
                path = "%s/%s" % (directory, cand)
                kind = self.fs.kind(path)
                if kind == "absent":
                    continue
                info = {"state": "installed", "path": path, "origin": "custom", "alias_of": None}
                resolved = path
                if kind == "symlink":
                    resolved, exists = self.fs.chase(path)
                    if self.fs.readlink(path) == "/dev/null" or resolved == "/dev/null":
                        info["state"] = "masked"
                        return info
                    if resolved is None or not exists:
                        info["state"] = "dangling"
                        return info
                    if os.path.basename(resolved) != cand:
                        info["alias_of"] = os.path.basename(resolved)
                    elif not resolved.startswith(tuple(d + "/" for d in UNIT_DIRS)):
                        info["state"] = "linked"
                    info["path"] = resolved
                elif kind == "file":
                    data, _ = self.fs.read_bytes(path, limit=1)
                    if data == b"":
                        info["state"] = "masked"
                        return info
                if resolved.startswith(PACKAGE_UNIT_DIRS):
                    info["origin"] = "package"
                return info
        return {"state": "not-installed", "path": None, "origin": None, "alias_of": None}

    def _load_unit(self, unit, info):
        """Parsed unit file merged with its /etc drop-ins, or None when unreadable."""
        if not info.get("path"):
            return None
        text, _ = self.fs.read_text(info["path"])
        if text is None:
            return None
        data = parse_unit(text)
        dirs = [self.dropin_dir(unit)]
        prefix, at, rest = unit.partition("@")
        if at and not rest.startswith("."):
            dirs.insert(0, self.dropin_dir("%s@.%s" % (prefix, rest.rpartition(".")[2])))
        for directory in dirs:
            names, _ = self.fs.listdir(directory)
            for name in names or ():
                if name.endswith(".conf"):
                    extra, _ = self.fs.read_text(directory + "/" + name)
                    if extra:
                        parse_unit(extra, into=data)
        return data

    def _boot_target(self):
        """The default.target systemd would pick from this tree at next boot."""
        for directory in UNIT_DIRS:
            path = directory + "/default.target"
            kind = self.fs.kind(path)
            if kind == "absent":
                continue
            if kind != "symlink":
                return {"source": path, "link": None, "effective": "default.target",
                        "resolvable": kind == "file"}
            resolved, exists = self.fs.chase(path)
            return {"source": path, "link": self.fs.readlink(path),
                    "effective": os.path.basename(resolved) if resolved else None,
                    "resolvable": bool(exists)}
        return {"source": None, "link": None, "effective": None, "resolvable": False}

    def _headless_link_text(self):
        for directory in UNIT_DIRS:
            path = "%s/%s" % (directory, HEADLESS_TARGET)
            resolved, exists = self.fs.chase(path)
            if (exists and os.path.basename(resolved) == HEADLESS_TARGET
                    and self.fs.kind(resolved) == "file"):
                return path
        raise ApplianceError(
            "%s not found under %s: this does not look like a systemd OS tree, nothing changed"
            % (HEADLESS_TARGET, self.fs.root))

    def _display_units(self):
        units = list(DISPLAY_MANAGERS)
        alias = ETC_UNITS + "/" + DISPLAY_MANAGER_ALIAS
        kind = self.fs.kind(alias)
        if kind == "symlink":
            target = os.path.basename(self.fs.readlink(alias) or "")
            if target.endswith(".service") and SUPPRESSIBLE_RE.match(target):
                units.append(target)
        elif kind == "file":
            units.append(DISPLAY_MANAGER_ALIAS)
        for unit in OTHER_DISPLAY_MANAGERS:
            if self._unit_info(unit)["state"] != "not-installed":
                units.append(unit)
        return [u for u in units if not protected_reason(u)]

    def _suppress_set(self, extra):
        for unit in extra:
            if not SUPPRESSIBLE_RE.match(unit) or len(unit) > 255:
                raise ApplianceError("not a suppressible unit name: %r" % unit)
            reason = protected_reason(unit)
            if reason:
                raise ApplianceError("refusing to suppress %s: preserved (%s)" % (unit, reason))
        ordered = []
        for unit in list(CONFLICTING_UNITS) + self._display_units() + list(extra):
            if unit not in ordered:
                ordered.append(unit)
        return ordered

    def _unit_names(self, unit):
        """Canonical and alias names referring to this systemd unit."""
        if self._aliases is None:
            self._aliases = {}
            seen = set()
            for directory in UNIT_DIRS:
                names, _ = self.fs.listdir(directory)
                for name in names or ():
                    if name in seen or not SUPPRESSIBLE_RE.fullmatch(name):
                        continue
                    seen.add(name)
                    path = directory + "/" + name
                    if self.fs.kind(path) != "symlink":
                        continue
                    resolved, _ = self.fs.chase(path)
                    if resolved is None:
                        continue
                    alias = os.path.basename(resolved)
                    if alias != name and SUPPRESSIBLE_RE.fullmatch(alias):
                        self._aliases.setdefault(name, set()).add(alias)
                        self._aliases.setdefault(alias, set()).add(name)
        found, pending = {unit}, [unit]
        while pending:
            other = self._aliases.get(pending.pop(), set()) - found
            found.update(other)
            pending.extend(other)
        return found

    def _protected_alias(self, unit):
        return next((name for name in sorted(self._unit_names(unit)) if protected_reason(name)), None)

    def _own_marker(self):
        return (self.fs.kind(MARKER) == "file"
                and self.fs.read_bytes(MARKER, limit=len(MARKER_TEXT)+1)[0] == MARKER_TEXT)

    def _scan_owned_dropins(self):
        """Units under /etc carrying a drop-in that is byte-for-byte ours."""
        if self.fs.kind(ETC_UNITS) != "dir":
            return []
        found = []
        names, _ = self.fs.listdir(ETC_UNITS)
        for name in names or ():
            unit = name[:-2]
            if not name.endswith(".d") or not SUPPRESSIBLE_RE.match(unit):
                continue
            if self.fs.kind(ETC_UNITS + "/" + name) != "dir":
                continue
            if self._dropin_status(unit) == "ok":
                found.append(unit)
        return found

    def _dropin_status(self, unit):
        """ok | missing | truncated | modified | collision for our drop-in of `unit`."""
        dkind = self.fs.kind(self.dropin_dir(unit))
        if dkind == "absent":
            return "missing"
        if dkind != "dir":
            return "collision"
        fkind = self.fs.kind(self.dropin_path(unit))
        if fkind == "absent":
            return "missing"
        if fkind != "file":
            return "collision"
        data, status = self.fs.read_bytes(self.dropin_path(unit), limit=len(DROPIN_TEXT) + 1)
        if status != "ok":
            return "collision"
        if data == DROPIN_TEXT:
            return "ok"
        return "truncated" if DROPIN_TEXT.startswith(data) else "modified"

    # ---- enable --------------------------------------------------------------------------

    def enable(self, extra=()):
        self._check_privilege()
        return self._locked(lambda: self._enable(tuple(extra)))

    def _enable(self, extra):
        self._aliases = None
        try:
            prior = self._load_state()
        except StateError as exc:
            raise ApplianceError("%s; run `normal` first to clean up" % exc)
        self._require_real_dirs(ETC_UNITS)
        link_text = self._headless_link_text()

        # Preflight: decide every step and find every collision before the first write.
        collisions = []
        for path, allowed in ((OWNED_DIR, ("absent", "dir")), (MARKER, ("absent", "file")),
                              (STATE, ("absent", "file"))):
            kind = self.fs.kind(path)
            if kind not in allowed:
                collisions.append("%s exists as a %s" % (path, kind))
        make_owned_dir = self.fs.kind(OWNED_DIR) == "absent"
        make_marker = self.fs.kind(MARKER) == "absent"
        if not make_marker and not self._own_marker():
            collisions.append("%s is not this tool's marker; left in place" % MARKER)

        link_kind = self.fs.kind(DEFAULT_LINK)
        if link_kind not in ("absent", "symlink"):
            collisions.append("%s is a %s, not a symlink; set the boot target back to a symlink "
                              "first" % (DEFAULT_LINK, link_kind))
        target = self._boot_target()
        relink = not (target["effective"] == HEADLESS_TARGET and target["resolvable"])
        if prior:
            default = dict(prior["default_target"])
            original_link = default["original"].get("target")
            current_link = self.fs.readlink(DEFAULT_LINK)
            if current_link and current_link not in (original_link, default.get("applied")):
                collisions.append("%s changed since enable; run normal before selecting a new boot target" % DEFAULT_LINK)
        else:
            original = {"kind": "absent"}
            if link_kind == "symlink":
                original = {"kind": "symlink", "target": self.fs.readlink(DEFAULT_LINK)}
            default = {"original": original, "applied": None}
        if relink:
            default["applied"] = link_text

        journaled = {e["unit"]: dict(e) for e in prior["dropins"]} if prior else {}
        plan, pending = [], []
        for unit in list(journaled) + [u for u in self._suppress_set(extra) if u not in journaled]:
            protected = self._protected_alias(unit)
            if protected:
                collisions.append("refusing to suppress %s: shares a unit with protected %s" % (unit, protected))
                continue
            status = self._dropin_status(unit)
            owned = unit in journaled
            if self.fs.kind(self.dropin_dir(unit)) not in ("absent", "dir"):
                collisions.append("%s exists as a %s, expected a directory" % (
                    self.dropin_dir(unit), self.fs.kind(self.dropin_dir(unit))))
                continue
            if status == "collision" or (status in ("modified", "truncated") and not owned):
                collisions.append("%s already exists and is not this tool's file"
                                  % self.dropin_path(unit))
                continue
            if status == "modified":
                collisions.append("%s was modified since it was created; remove it or run "
                                  "`normal`" % self.dropin_path(unit))
                continue
            entry = journaled.get(unit) or {"unit": unit, "dir_created": False,
                                            "adopted": status == "ok"}
            make_dir = self.fs.kind(self.dropin_dir(unit)) == "absent"
            if make_dir:
                entry["dir_created"] = True
            action = {"ok": "keep", "missing": "create", "truncated": "rewrite"}[status]
            plan.append((entry, make_dir, action))
            if not owned:
                pending.append(entry)
        if collisions:
            raise ApplianceError("path collision, nothing changed:\n  " + "\n  ".join(collisions))

        if (prior and prior["phase"] == "enabled" and not pending and not relink
                and not make_marker and all(action == "keep" for _, _, action in plan)):
            return {"result": "already-enabled", "warnings": []}

        state = {"tool": TOOL, "schema": SCHEMA, "phase": "enabling", "default_target": default,
                 "dropins": [entry for entry, _, _ in plan],
                 "own_dir_created": prior.get("own_dir_created", True) if prior else make_owned_dir}
        try:
            if make_owned_dir:
                self._mkdir(OWNED_DIR)
            self._save_state(state)
            for entry, make_dir, action in plan:
                if make_dir:
                    self._mkdir(self.dropin_dir(entry["unit"]))
                if action == "create":
                    self._write_new(self.dropin_path(entry["unit"]), DROPIN_TEXT)
                elif action == "rewrite":
                    self._write_atomic(self.dropin_path(entry["unit"]), DROPIN_TEXT, "repair")
            if relink:
                self._symlink_atomic(DEFAULT_LINK, link_text)
            if make_marker:
                self._write_new(MARKER, MARKER_TEXT)  # commit point: suppression becomes active
            state["phase"] = "enabled"
            self._save_state(state)
            if not self.dry_run:
                self._verify_written(state)
        except BaseException as exc:
            leftovers = self._rollback(state, prior, pending)
            if not isinstance(exc, (OSError, ApplianceError)):
                raise
            reason = _strerror(exc) if isinstance(exc, OSError) else str(exc)
            if isinstance(exc, OSError) and exc.filename:
                reason += " (%s)" % exc.filename
            if leftovers:
                raise ApplianceError(
                    "enable failed: %s\nrollback incomplete, run `normal` to finish:\n  %s"
                    % (reason, "\n  ".join(leftovers)))
            raise ApplianceError("enable failed and was rolled back: " + reason)
        return {"result": "dry-run" if self.dry_run else "enabled", "warnings": []}

    def _verify_written(self, state):
        bad = [e["unit"] for e in state["dropins"] if self._dropin_status(e["unit"]) != "ok"]
        target = self._boot_target()
        if bad:
            raise ApplianceError("drop-in verification failed for " + ", ".join(bad))
        if self.fs.kind(MARKER) != "file":
            raise ApplianceError("marker verification failed")
        if target["effective"] != HEADLESS_TARGET or not target["resolvable"]:
            raise ApplianceError("boot target verification failed (%s)" % target["effective"])

    def _rollback(self, state, prior, pending):
        """Return to the pre-run state after an in-process failure; returns what could not be undone."""
        try:
            if prior is not None and prior["phase"] == "enabled":
                # A repair of an already-enabled system failed: stay enabled, drop this run's additions.
                errors, warnings = [], []
                for entry in pending:
                    if not entry["adopted"]:
                        self._remove_dropin(entry, errors, warnings)
                # Keep this run's boot-target record: the link may already have been re-applied.
                self._save_state(dict(prior, default_target=state["default_target"]))
                return errors
            errors, _ = self._restore(state, rollback=True)
            return errors
        except OSError as exc:
            return ["rollback: " + _strerror(exc)]

    # ---- normal --------------------------------------------------------------------------

    def normal(self):
        self._check_privilege()
        return self._locked(self._normal)

    def _normal(self):
        corrupt = None
        try:
            state = self._load_state()
        except StateError as exc:
            state, corrupt = None, str(exc)
        self._require_real_dirs(ETC_UNITS)
        if corrupt and not self._own_marker() and not self._scan_owned_dropins():
            raise ApplianceError("Unrecognized journal at %s without an owned marker or drop-ins; left in place." % STATE)
        if corrupt and self.fs.kind(STATE + ".unreadable") != "absent":
            raise ApplianceError("Recovery destination %s.unreadable already exists; left in place." % STATE)
        owned_before = (state is not None or corrupt is not None
                        or self.fs.kind(MARKER) != "absent" or bool(self._scan_owned_dropins()))
        errors, warnings = self._restore(state)
        if corrupt:
            target = self._boot_target()
            try:
                self.actions.append("set aside " + STATE + ".unreadable")
                if not self.dry_run:
                    os.replace(self.fs.host(STATE), self.fs.host(STATE + ".unreadable"))
            except OSError as exc:
                errors.append("set aside %s: %s" % (STATE, _strerror(exc)))
            errors.append(
                "%s; marker and owned drop-ins were removed, but the original boot target is "
                "unknown and was NOT restored (next boot: %s). Set it with "
                "`systemctl set-default <target>` if needed." % (corrupt, target["effective"]))
        result = "dry-run" if self.dry_run else ("normal" if owned_before else "already-normal")
        return {"result": "failed" if errors else result, "errors": errors, "warnings": warnings}

    def _restore(self, state, rollback=False):
        """Undo everything owned. Idempotent; keeps the journal when a step fails so it can be re-run."""
        errors, warnings = [], []

        def attempt(what, fn, *args):
            try:
                fn(*args)
            except OSError as exc:
                errors.append("%s: %s" % (what, _strerror(exc)))

        owned_kind = self.fs.kind(OWNED_DIR)
        if owned_kind not in ("absent", "dir"):
            errors.append("%s is a %s, expected a directory; not touching it" % (OWNED_DIR, owned_kind))
        if state is not None and owned_kind == "dir" and self.fs.kind(STATE) == "file":
            state["phase"] = "restoring"
            attempt("journal " + STATE, self._save_state, state)

        # Marker first: once it is gone every drop-in is inert, whatever happens next.
        kind = self.fs.kind(MARKER) if owned_kind == "dir" else "absent"
        owned_marker = kind == "file" and self._own_marker()
        if owned_marker:
            attempt("remove " + MARKER, self._unlink, MARKER)
        elif kind != "absent":
            errors.append("%s is not this tool's marker; left in place" % MARKER)

        if state is not None and state["default_target"].get("applied") is not None:
            default = state["default_target"]
            current = self.fs.readlink(DEFAULT_LINK) if self.fs.kind(DEFAULT_LINK) == "symlink" else None
            if current == default["applied"]:
                if default["original"]["kind"] == "absent":
                    attempt("remove " + DEFAULT_LINK, self._unlink, DEFAULT_LINK)
                elif default["original"]["target"] != current:
                    attempt("restore " + DEFAULT_LINK, self._symlink_atomic, DEFAULT_LINK,
                            default["original"]["target"])
            elif not rollback:
                warnings.append("%s was changed after enable (now %s); left as it is"
                                % (DEFAULT_LINK, current or self.fs.kind(DEFAULT_LINK)))

        entries = {e["unit"]: e for e in state["dropins"]} if state is not None else {}
        if not rollback:
            for unit in self._scan_owned_dropins():
                entries.setdefault(unit, {"unit": unit, "dir_created": False, "adopted": False})
        for entry in entries.values():
            if not (rollback and entry["adopted"]):
                self._remove_dropin(entry, errors, warnings)

        if not errors:
            if state is not None and self.fs.kind(STATE) == "file":
                attempt("remove " + STATE, self._unlink, STATE)
            if (self.fs.kind(OWNED_DIR) == "dir" and
                    (state.get("own_dir_created", True) if state else owned_marker)):
                attempt("remove " + OWNED_DIR, self._rmdir, OWNED_DIR)
        return errors, warnings

    def _remove_dropin(self, entry, errors, warnings):
        directory, path = self.dropin_dir(entry["unit"]), self.dropin_path(entry["unit"])
        try:
            kind = self.fs.kind(directory)
            if kind != "dir":
                if kind != "absent":
                    warnings.append("%s is a %s, not a directory; left in place" % (directory, kind))
                return
            status = self._dropin_status(entry["unit"])
            if status in ("ok", "truncated"):
                self._unlink(path)
            elif status != "missing":
                warnings.append("%s is not what this tool wrote (%s); left in place" % (path, status))
            if entry["dir_created"]:
                self._rmdir(directory)
        except OSError as exc:
            errors.append("remove %s: %s" % (path, _strerror(exc)))

    # ---- status --------------------------------------------------------------------------

    def _systemctl_default(self):
        """Best-effort cross-check with `systemctl --root=<root> get-default` (file-only, read-only)."""
        exe = self.systemctl
        if exe == "auto":
            exe = next((c for c in SYSTEMCTL_CANDIDATES if os.access(c, os.X_OK)), None)
        if not exe:
            return {"available": False}
        argv = [exe, "--root=" + self.fs.root, "get-default"]
        try:
            proc = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, timeout=15, env=SYSTEMCTL_ENV,
                                  check=False)
        except (OSError, subprocess.SubprocessError) as exc:
            return {"available": True, "default": None, "error": type(exc).__name__}
        name = proc.stdout.decode("utf-8", "replace").strip()
        if proc.returncode != 0 or not re.match(r"^[A-Za-z0-9:_.\\@-]+\.target$", name):
            return {"available": True, "default": None, "error": "exit %d" % proc.returncode}
        return {"available": True, "default": name}

    def _cmdline_tokens(self):
        tokens = []
        for path in ("/boot/firmware/cmdline.txt", "/boot/cmdline.txt"):
            text, _ = self.fs.read_text(path)
            for tok in (text or "").split():
                key, eq, value = tok.partition("=")
                if eq and key in ("systemd.unit", "systemd.wants", "init"):
                    shown = key + "=" + (value if SAFE_TOKEN_RE.match(value) else "<redacted>")
                    overrides = key == "systemd.unit" and value != HEADLESS_TARGET
                elif tok in ("1", "5", "s", "S", "single", "rescue", "emergency"):
                    shown, overrides = tok, True
                else:
                    continue
                tokens.append({"path": path, "token": shown, "overrides_target": overrides})
        return tokens

    def inspect(self, audit=True):
        """Read-only report of the tree: mode, problems, suppressed units, launch surfaces."""
        self._enabled_cache = self._aliases = None
        fs = self.fs
        pretty = None
        text, _ = fs.read_text("/etc/os-release")
        match = re.search(r'^PRETTY_NAME="?([^"\n]{1,80})"?$', text or "", re.M)
        if match:
            pretty = match.group(1)
        report = {
            "tool": TOOL,
            "scope": {"root": fs.root, "live": fs.live, "os_release": pretty,
                      "note": "files under this root only; the running system was not queried"},
            "mode": "unknown", "problems": [], "suppressed": [], "audit": None,
        }
        problems = report["problems"]
        if fs.kind(ETC_UNITS) != "dir":
            problems.append("%s is not a directory under %s: not a systemd OS tree, nothing "
                            "inspected" % (ETC_UNITS, fs.root))
            return report

        state, phase = None, None
        try:
            state = self._load_state()
            phase = state["phase"] if state else None
            journal = "absent" if state is None else "ok"
        except StateError as exc:
            journal = "unreadable"
            problems.append(str(exc))
        marker = fs.kind(MARKER)
        target = self._boot_target()
        target["cmdline"] = self._cmdline_tokens()
        target["original"] = state["default_target"]["original"] if state else None
        target["systemctl"] = self._systemctl_default()
        report.update({"journal": journal, "phase": phase,
                       "marker": marker != "absent", "boot_target": target})

        units = [e["unit"] for e in state["dropins"]] if state else []
        units += [u for u in self._scan_owned_dropins() if u not in units]
        enabled = self._enabled_map()
        for unit in units:
            info = self._unit_info(unit)
            row = {"unit": unit, "dropin": self._dropin_status(unit), "unit_file": info["state"],
                   "unit_path": info["path"], "enabled_in": enabled.get(unit, []),
                   "protected_alias": self._protected_alias(unit), "condition_reset_by": self._condition_resets(unit)}
            report["suppressed"].append(row)

        if journal == "absent" and marker == "absent" and not units:
            report["mode"] = "normal"
        else:
            if journal == "absent":
                problems.append("marker or drop-ins exist without a journal; run `normal` to clean up")
            elif phase not in (None, "enabled"):
                problems.append("journal phase is %r (interrupted run); re-run `enable` or `normal`"
                                % phase)
            if marker == "absent":
                problems.append("marker %s is missing: the suppression drop-ins are inert" % MARKER)
            elif marker != "file":
                problems.append("marker %s is a %s, expected a regular file" % (MARKER, marker))
            elif not self._own_marker():
                problems.append("marker %s was replaced with foreign contents" % MARKER)
            for row in report["suppressed"]:
                if row["protected_alias"]:
                    problems.append("%s shares its suppression with protected %s" % (row["unit"], row["protected_alias"]))
                if row["dropin"] != "ok":
                    problems.append("drop-in for %s is %s" % (row["unit"], row["dropin"]))
                for name in row["condition_reset_by"]:
                    problems.append("%s can clear or supersede the appliance condition for %s"
                                    % (name, row["unit"]))
            if target["effective"] != HEADLESS_TARGET:
                problems.append("next boot target is %s, not %s"
                                % (target["effective"], HEADLESS_TARGET))
            elif not target["resolvable"]:
                problems.append("%s does not resolve to a unit file" % target["source"])
            for tok in target["cmdline"]:
                if tok["overrides_target"]:
                    problems.append("kernel command line in %s overrides the boot target (%s)"
                                    % (tok["path"], tok["token"]))
            reported = target["systemctl"].get("default")
            if reported and reported != target["effective"]:
                problems.append("systemctl --root reports default target %s, files say %s"
                                % (reported, target["effective"]))
            report["mode"] = "degraded" if problems else "appliance"
        if audit:
            report["audit"] = Audit(self, set(units)).run()
        return report

    def _condition_resets(self, unit):
        """Identify later condition resets across the unit names systemd considers.

        Equal-name overrides on alias/template paths can have subtle precedence.
        Report conflicting customizations for operator review.
        """
        kind = unit.rpartition(".")[2]
        scopes = {kind}
        for name in self._unit_names(unit):
            scopes.add(name)
            base = name.rpartition(".")[0]
            if "@" in base:
                scopes.add(base.partition("@")[0] + "@." + kind)
            segments = base.partition("@")[0].split("-")
            scopes.update("-".join(segments[:i]) + "-." + kind for i in range(1,len(segments)))
        uncertain = set()
        for scope in scopes:
            claimed = set()
            for directory in UNIT_DIRS:
                parent = directory + "/" + scope + ".d"
                names, status = self.fs.listdir(parent)
                if status == "unreadable":
                    uncertain.add(parent)
                for name in names or ():
                    if name in claimed or not name.endswith(".conf") or name < DROPIN_NAME:
                        continue
                    claimed.add(name)
                    path = parent + "/" + name
                    if path == self.dropin_path(unit):
                        continue
                    text, status = self.fs.read_text(path)
                    if self.fs.chase(path)[0] == "/dev/null":
                        continue
                    if status != "ok" or (name == DROPIN_NAME and text != DROPIN_TEXT.decode()):
                        uncertain.add(path)
                        continue
                    section = None
                    for line in text.splitlines():
                        line = line.strip()
                        if line.startswith("[") and line.endswith("]"):
                            section = line
                        elif section == "[Unit]" and CONDITION_RESET_RE.fullmatch(line):
                            uncertain.add(path)
                            break
        return sorted(uncertain)


class Audit:
    """Inventory of places in the tree that launch executables at boot or login.

    Levels: review (likely workload outside the covered units), kept (preserved by policy),
    info (boot surface, nothing suspicious seen), desktop (only runs in a graphical session),
    covered (suppressed in appliance mode). Only paths, unit names, account names and program
    paths are reported; arguments, environment values and file contents never are.
    """

    HOME_STARTUP = (".profile", ".bash_profile", ".bash_login", ".bashrc", ".zprofile", ".zlogin")

    def __init__(self, app, suppressed):
        self.app = app
        self.fs = app.fs
        # What appliance mode covers: units already suppressed, or what `enable` would suppress.
        self.covered = set(suppressed) or set(app._suppress_set(()))
        self.items = []
        self.unreadable = []
        self.quiet_units = 0
        self.autologin_users = set()

    def add(self, level, surface, where, detail):
        self.items.append({"level": level, "surface": surface, "where": where, "detail": detail})

    def read(self, path):
        text, status = self.fs.read_text(path)
        if status == "unreadable":
            self.unreadable.append(path)
        return text

    def ls(self, path):
        names, status = self.fs.listdir(path)
        if status == "unreadable":
            self.unreadable.append(path)
        return names or []

    def run(self):
        candidates = [("/home/" + n, n) for n in self.ls("/home")[:20]] + [("/root", "root")]
        self.homes = []
        for home, user in candidates:
            if self.fs.kind(home) != "dir":
                continue
            if self.fs.listdir(home)[1] == "ok":
                self.homes.append((home, user))
            else:
                self.unreadable.append(home)
        self.units()
        self.kernel_cmdline()
        self.autologin()
        self.shell_startup()
        self.rc_local()
        self.cron()
        self.user_units()
        self.desktop_autostart()
        order = {"review": 0, "kept": 1, "info": 2, "desktop": 3, "covered": 4}
        self.items.sort(key=lambda item: order[item["level"]])
        return {
            "items": self.items,
            "homes_inspected": [home for home, _ in self.homes],
            "package_units_not_listed": self.quiet_units,
            "unreadable": sorted(set(self.unreadable)),
            "not_inspected": list(NOT_INSPECTED),
        }

    # -- systemd system units --

    def units(self):
        app = self.app
        enabled = app._enabled_map()
        triggers = {name: list(targets) for name, targets in enabled.items()}
        names = set(triggers) | self.covered
        for name in self.ls(ETC_UNITS):
            if SUPPRESSIBLE_RE.match(name) and self.fs.kind(ETC_UNITS + "/" + name) != "dir":
                names.add(name)
        # Follow aliases and socket/timer/path activation to the service that actually executes.
        for name in sorted(names):
            info = app._unit_info(name)
            if info["alias_of"]:
                triggers.setdefault(info["alias_of"], []).append("alias " + name)
                names.add(info["alias_of"])
            elif not name.endswith(".service") and name in enabled and info["state"] in (
                    "installed", "linked"):
                data = app._load_unit(name, info) or {}
                section, key = {"socket": ("Socket", "Service")}.get(
                    name.rpartition(".")[2], (name.rpartition(".")[2].capitalize(), "Unit"))
                service = (data.get(section, {}).get(key) or [name.rpartition(".")[0] + ".service"])[-1]
                if SUPPRESSIBLE_RE.match(service):
                    triggers.setdefault(service, []).append("via " + name)
                    names.add(service)
        for name in sorted(names):
            if name.endswith(".service"):
                self.service(name, triggers.get(name, []))
        for name in sorted(triggers):
            if any(fnmatch.fnmatchcase(name, pat) for pat in SUPERVISORS):
                self.add("review", "supervisor", name,
                         "enabled process supervisor; it can start workloads this inventory cannot see")

    def service(self, name, triggers):
        info = self.app._unit_info(name)
        if name in self.covered:
            where = {"not-installed": "not installed (pre-emptive)"}.get(info["state"], info["state"])
            self.add("covered", "unit", name, "suppressed in appliance mode; unit file: %s%s" % (
                where, "; enabled in: " + ", ".join(triggers) if triggers else ""))
            return
        if info["state"] in ("not-installed", "masked") or info["alias_of"]:
            return
        custom = info["origin"] == "custom"
        if info["state"] == "dangling":
            if triggers:
                self.add("info", "unit", name, "enabled but its unit file link is broken")
            return
        data = self.app._load_unit(name, info)
        heads, reasons = [], []
        if data is None:
            self.unreadable.append(info["path"])
            reasons.append("unit file unreadable")
        else:
            service = data.get("Service", {})
            for key in ("ExecStartPre", "ExecStart", "ExecStartPost"):
                for value in service.get(key, []):
                    head = command_head(value)
                    if head["head"] and head["head"] not in heads:
                        heads.append(head["head"])
                    reasons += [r for r in head["reasons"] if r not in reasons]
        if HINT_RE.search(name) and "robot-workload hint" not in reasons:
            reasons.append("robot-workload hint")
        if not triggers and not custom:
            return
        detail = "%s; exec: %s" % (
            "enabled in: " + ", ".join(triggers) if triggers else "installed, not enabled",
            ", ".join(heads[:3]) or "unknown")
        if custom:
            detail += "; custom unit (%s)" % info["path"]
        keep = protected_reason(name)
        if not triggers:
            level = "info"
        elif reasons and keep:
            level, detail = "kept", detail + "; preserved by policy (%s)" % keep
        elif reasons and all(t == "graphical.target" for t in triggers):
            level, detail = "desktop", detail + "; only pulled in by graphical.target"
        elif reasons:
            level = "review"
        elif custom:
            level = "info"
        else:
            self.quiet_units += 1
            return
        if reasons:
            detail += "; " + ", ".join(reasons)
        self.add(level, "unit", name, detail)

    # -- everything that is not a system unit --

    def kernel_cmdline(self):
        for tok in self.app._cmdline_tokens():
            if tok["token"].startswith(("init=", "systemd.wants=")):
                self.add("review", "kernel-cmdline", tok["path"], "boot parameter " + tok["token"])

    def autologin(self):
        for name in self.ls(ETC_UNITS):
            match = re.match(r"^((?:serial-)?getty@.*)\.service\.d$", name)
            if not match:
                continue
            for conf in self.ls(ETC_UNITS + "/" + name):
                text = self.read("%s/%s/%s" % (ETC_UNITS, name, conf)) if conf.endswith(".conf") else None
                found = re.search(r"^\s*ExecStart\s*=.*?(?:--autologin|\s-a)[ =]+(\S+)", text or "", re.M)
                if found:
                    user = found.group(1) if USER_RE.match(found.group(1)) else "<redacted>"
                    self.autologin_users.add(user)
                    self.add("info", "console-autologin", "%s/%s/%s" % (ETC_UNITS, name, conf),
                             "user %s is logged in on %s at boot (also headless): that account's shell "
                             "startup files and enabled user units run" % (user, match.group(1)))
        paths = ["/etc/lightdm/lightdm.conf"] + [
            "/etc/lightdm/lightdm.conf.d/" + n for n in self.ls("/etc/lightdm/lightdm.conf.d")
            if n.endswith(".conf")]
        for path in paths:
            found = re.search(r"^\s*autologin-user\s*=\s*(\S+)", self.read(path) or "", re.M)
            if found:
                user = found.group(1) if USER_RE.match(found.group(1)) else "<redacted>"
                self.add("desktop", "desktop-autologin", path,
                         "display manager logs in user %s; inactive while the display manager "
                         "is suppressed" % user)

    def shell_startup(self):
        files = [("/etc/profile", None), ("/etc/bash.bashrc", None)]
        files += [("/etc/profile.d/" + n, None) for n in self.ls("/etc/profile.d") if n.endswith(".sh")]
        for home, user in self.homes:
            files += [("%s/%s" % (home, name), user) for name in self.HOME_STARTUP]
        for path, user in files:
            text = self.read(path)
            if text is None:
                continue
            _, flagged = scan_lines(text, skip_env=True)
            if flagged:
                when = "runs at every login including SSH"
                if (user is None and self.autologin_users) or user in self.autologin_users:
                    when += " and at boot via console autologin"
                self.add("review", "shell-startup", path, "line(s) %s match robot-workload hints; %s"
                         % (", ".join(map(str, flagged)), when))

    def rc_local(self):
        text = self.read("/etc/rc.local")
        if text is None:
            return
        lines = [l for l in text.splitlines() if l.strip() and not re.match(r"^\s*(#|exit\s+0\s*$)", l)]
        _, flagged = scan_lines(text)
        if not lines:
            return
        if not self.fs.executable("/etc/rc.local"):
            self.add("info", "rc.local", "/etc/rc.local",
                     "%d active line(s) but not executable, so not run at boot" % len(lines))
            return
        detail = "runs as root at boot: %d active line(s)" % len(lines)
        if flagged:
            detail += "; line(s) %s match robot-workload hints" % ", ".join(map(str, flagged))
        self.add("review" if flagged else "info", "rc.local", "/etc/rc.local", detail)

    def cron(self):
        tables = [("/etc/crontab", True)]
        tables += [("/etc/cron.d/" + n, True) for n in self.ls("/etc/cron.d") if not n.startswith(".")]
        tables += [("/var/spool/cron/crontabs/" + n, False) for n in self.ls("/var/spool/cron/crontabs")]
        for path, has_user in tables:
            text = self.read(path)
            periodic = 0
            for number, line in enumerate((text or "").splitlines(), 1):
                fields = line.split()
                if not fields or fields[0].startswith("#") or re.match(r"^[A-Za-z_]\w*\s*=", line.strip()):
                    continue
                skip = (1 if fields[0].startswith("@") else 5) + (1 if has_user else 0)
                if len(fields) <= skip:
                    continue
                head = command_head(line.strip().split(None, skip)[skip])
                at_boot = fields[0] == "@reboot"
                if not at_boot and not head["reasons"]:
                    periodic += 1
                    continue
                detail = "%s entry: %s" % ("@reboot" if at_boot else "scheduled", head["head"])
                if head["reasons"]:
                    detail += "; " + ", ".join(head["reasons"])
                self.add("review" if head["reasons"] else "info", "cron", "%s:%d" % (path, number), detail)
            if periodic:
                self.add("info", "cron", path, "%d scheduled entr%s with nothing flagged"
                         % (periodic, "y" if periodic == 1 else "ies"))

    def user_units(self):
        for name in self.ls("/var/lib/systemd/linger"):
            if USER_RE.match(name):
                self.add("info", "user-linger", "/var/lib/systemd/linger/" + name,
                         "user manager for %s starts at boot without a login; its enabled user "
                         "units run headless" % name)
        bases = [("/etc/systemd/user", None)] + [(h + "/.config/systemd/user", u) for h, u in self.homes]
        for base, user in bases:
            for scope in self.ls(base):
                if not scope.endswith(".service.d"):
                    continue
                for filename in self.ls(base + "/" + scope):
                    if not filename.endswith(".conf"):
                        continue
                    path = "/".join((base, scope, filename))
                    service = parse_unit(self.read(path) or "").get("Service", {})
                    for key in ("ExecCondition", "ExecStartPre", "ExecStart", "ExecStartPost"):
                        for value in service.get(key, []):
                            head = command_head(value)
                            if head["reasons"]:
                                self.add("review", "user-unit-dropin", path,
                                         "%s overrides %s for %s when the user manager activates it"
                                         % (head["head"], key, scope[:-2]))
            for wants in self.ls(base):
                if not wants.endswith((".wants", ".requires")):
                    continue
                for unit in self.ls(base + "/" + wants):
                    link = "%s/%s/%s" % (base, wants, unit)
                    resolved, exists = self.fs.chase(link)
                    packaged = bool(resolved) and resolved.startswith(
                        ("/usr/lib/systemd/user/", "/lib/systemd/user/"))
                    text = self.read(link) if exists else None
                    heads, reasons = [], []
                    for value in parse_unit(text or "").get("Service", {}).get("ExecStart", []):
                        head = command_head(value)
                        heads.append(head["head"])
                        reasons += [r for r in head["reasons"] if r not in reasons]
                    if HINT_RE.search(unit) and "robot-workload hint" not in reasons:
                        reasons.append("robot-workload hint")
                    if packaged and not reasons:
                        continue
                    detail = ("user unit enabled in %s%s; exec: %s; starts with the user manager "
                              "(login, autologin or linger)" % (
                                  wants.rpartition(".")[0],
                                  " for " + user if user else " for all users",
                                  ", ".join(heads[:2]) or "unknown"))
                    if reasons:
                        detail += "; " + ", ".join(reasons)
                    self.add("review" if reasons else "info", "user-unit", link, detail)

    def desktop_autostart(self):
        dirs = [("/etc/xdg/autostart", False)] + [(h + "/.config/autostart", True) for h, _ in self.homes]
        for directory, list_all in dirs:
            quiet = 0
            for name in self.ls(directory):
                if not name.endswith(".desktop"):
                    continue
                entry = parse_unit(self.read(directory + "/" + name) or "").get("Desktop Entry", {})
                if (entry.get("Hidden") or [""])[-1].lower() == "true" or not entry.get("Exec"):
                    continue
                head = command_head(entry["Exec"][-1])
                if head["reasons"] or list_all:
                    self.add("desktop", "desktop-autostart", directory + "/" + name,
                             "exec: %s%s" % (head["head"], "; " + ", ".join(head["reasons"])
                                             if head["reasons"] else ""))
                else:
                    quiet += 1
            if quiet:
                self.add("desktop", "desktop-autostart", directory,
                         "%d entr%s with nothing flagged" % (quiet, "y" if quiet == 1 else "ies"))
        scripts = ["/etc/xdg/labwc/autostart"]
        for home, _ in self.homes:
            scripts += [home + "/.config/labwc/autostart", home + "/.config/wayfire.ini"]
            scripts += ["%s/.config/lxsession/%s/autostart" % (home, n)
                        for n in self.ls(home + "/.config/lxsession")]
        for path in scripts:
            text = self.read(path)
            if text is None:
                continue
            if path.endswith(".ini"):
                text = "\n".join(v for vals in parse_unit(text).get("autostart", {}).values() for v in vals)
            active, flagged = scan_lines(text)
            if active and (flagged or not path.startswith("/etc/")):
                self.add("desktop", "desktop-autostart", path, "%d active line(s)%s" % (
                    active, "; %d match robot-workload hints" % len(flagged) if flagged else ""))


# ---- rendering and CLI -------------------------------------------------------------------


def render(report):
    scope = report["scope"]
    lines = ["fc-appliance boot personality: %s" % report["mode"].upper()]
    lines.append("scope: %s %s; os-release: %s; %s" % (
        "live root" if scope["live"] else "offline tree", scope["root"],
        scope["os_release"] or "not identified", scope["note"]))
    target = report.get("boot_target")
    if target:
        line = "next boot target: %s" % (target["effective"] or "none found")
        if target["source"]:
            line += " (%s%s)" % (target["source"], " -> " + target["link"] if target["link"] else "")
        lines.append(line)
        if target["original"]:
            original = target["original"]
            lines.append("original target restored by `normal`: %s" % (
                original["target"] if original["kind"] == "symlink"
                else "no %s (distribution default)" % DEFAULT_LINK))
        check = target["systemctl"]
        lines.append("systemctl --root cross-check: %s" % (
            "not available on this host, skipped" if not check["available"]
            else check.get("default") or "failed (%s)" % check.get("error")))
    for problem in report["problems"]:
        lines.append("PROBLEM: " + problem)
    if report["suppressed"]:
        lines.append("suppressed units (drop-in %s keyed on %s%s):" % (
            DROPIN_NAME, MARKER, "" if report.get("marker") else ", MARKER ABSENT"))
        for row in report["suppressed"]:
            lines.append("  %-24s drop-in %-9s unit file: %s%s" % (
                row["unit"], row["dropin"], row["unit_file"],
                "; enabled in " + ", ".join(row["enabled_in"]) if row["enabled_in"] else ""))
    audit = report.get("audit")
    if audit:
        remaining = [i for i in audit["items"] if i["level"] != "covered"]
        covered = [i for i in audit["items"] if i["level"] == "covered"]
        if report["mode"] != "appliance" and covered:
            lines.append("covered when appliance mode is enabled: "
                         + ", ".join(i["where"] for i in covered))
        lines.append("remaining boot/login launch surfaces found in this tree (%d to review):"
                     % sum(1 for i in remaining if i["level"] == "review"))
        for item in remaining:
            lines.append("  %-7s %s %s: %s" % (
                item["level"].upper(), item["surface"], item["where"], item["detail"]))
        if not remaining:
            lines.append("  none found")
        lines.append("  %d enabled package-provided unit(s) with system executables not listed"
                     % audit["package_units_not_listed"])
        lines.append("homes inspected: %s" % (", ".join(audit["homes_inspected"]) or "none"))
        for path in audit["unreadable"]:
            lines.append("NOT INSPECTED (unreadable, re-run as root): " + path)
        for note in audit["not_inspected"]:
            lines.append("not inspected: " + note)
    return "\n".join(lines)


def _add_common(parser, top):
    default = {} if top else {"default": argparse.SUPPRESS}
    parser.add_argument("--root", metavar="DIR", **({"default": "/"} if top else default),
                        help="operate on an offline OS tree instead of the live system")
    parser.add_argument("--json", action="store_true", **default, help="machine-readable output")


def build_parser():
    parser = argparse.ArgumentParser(
        prog="fc_appliance.py",
        description="Switch the next boot between the headless FC appliance and the normal image.")
    _add_common(parser, top=True)
    sub = parser.add_subparsers(dest="command", required=True)
    enable = sub.add_parser("enable", help="next boot: headless appliance, robot units suppressed")
    enable.add_argument("--also-suppress", metavar="UNIT", action="append", default=[],
                        help="additional unit to suppress (repeatable)")
    normal = sub.add_parser("normal", help="undo enable: restore boot target, remove owned files")
    for cmd in (enable, normal):
        cmd.add_argument("--dry-run", action="store_true", help="show what would change, write nothing")
    status = sub.add_parser("status", help="report mode and remaining boot launch surfaces")
    status.add_argument("--no-audit", action="store_true", help="skip the launch surface inventory")
    for cmd in (enable, normal, status):
        _add_common(cmd, top=False)
    return parser


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None, out=None, err=None):
    out, err = out or sys.stdout, err or sys.stderr
    try:
        args = build_parser().parse_args(argv)
    except SystemExit as exc:
        return exc.code if isinstance(exc.code, int) else EXIT_USAGE
    if not os.path.isdir(args.root):
        print("fc_appliance: --root %s is not a directory" % args.root, file=err)
        return EXIT_USAGE
    app = FcAppliance(args.root, dry_run=getattr(args, "dry_run", False))
    payload = {"command": args.command}
    code = EXIT_OK
    old_handlers = {}
    try:
        for signum in (signal.SIGTERM, signal.SIGHUP):  # roll back instead of dying mid-write
            try:
                old_handlers[signum] = signal.signal(signum, _interrupt)
            except ValueError:
                pass
        if args.command == "enable":
            payload.update(app.enable(args.also_suppress))
        elif args.command == "normal":
            payload.update(app.normal())
            if payload["errors"]:
                code = EXIT_FAIL
        report = app.inspect(audit=not getattr(args, "no_audit", False))
        if args.command == "status" and report["mode"] in ("degraded", "unknown"):
            code = EXIT_DEGRADED
        if payload.get("result") in ("enabled", "already-enabled") and report["mode"] != "appliance":
            code = EXIT_DEGRADED
    except PrivilegeError as exc:
        print("fc_appliance: " + str(exc), file=err)
        return EXIT_PERM
    except ApplianceError as exc:
        print("fc_appliance: " + str(exc), file=err)
        if args.json:
            json.dump(dict(payload, result="failed", error=str(exc), actions=app.actions), out, indent=2)
            out.write("\n")
        return EXIT_FAIL
    except KeyboardInterrupt:
        print("fc_appliance: interrupted; changes of this run were rolled back where possible, "
              "check `status`", file=err)
        return EXIT_FAIL
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)

    if args.json:
        json.dump(dict(payload, actions=app.actions, status=report), out, indent=2)
        out.write("\n")
        return code
    if args.command != "status":
        print("%s: %s" % (args.command, payload["result"]), file=out)
        for action in app.actions:
            print(("  would " if app.dry_run else "  ") + action, file=out)
        for warning in payload.get("warnings", []):
            print("WARNING: " + warning, file=out)
        for error in payload.get("errors", []):
            print("ERROR: " + error, file=out)
        if payload["result"] in ("enabled", "normal"):
            print("Takes full effect at next boot. Nothing was started, stopped, signalled or "
                  "reloaded, and unit enablement and network configuration are unchanged; a "
                  "suppressed unit that is (re)started before the reboot may already be skipped."
                  if payload["result"] == "enabled" else
                  "The boot target change takes effect at next boot. Nothing was started, stopped, "
                  "signalled or reloaded.", file=out)
    print(render(report), file=out)
    return code


if __name__ == "__main__":
    sys.exit(main())
