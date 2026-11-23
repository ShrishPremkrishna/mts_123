"""One-use, explicitly enabled localhost credentials for the owned onboard page.

The operator's kiosk launcher writes a short-lived grant in a private directory;
only its nonce hash is stored. The existing API consumes it once. No provider or
robot lifecycle operations belong here, and no credential is persisted by this module.
"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import stat
import time
from urllib.parse import urlsplit

SCHEMA = 'pupper_onboard_bootstrap_v1'
TTL_SECONDS = 60
NONCE = re.compile(r'[0-9a-f]{64}\Z')
CONFIG = {'voiceInputMode': 'hands_free', 'robotApiUrl': '', 'cameraSource': 'server',
          'eyeTrackingSource': 'server', 'enableAudio': True, 'enableVideo': True,
          'modelName': 'gemini-robotics-er-2-streaming-preview',
          'ttsModelName': 'gemini-3.1-flash-tts-preview', 'voiceName': 'Puck',
          'outputPolicy': 'conversation', 'quickReceipts': True}
MODEL_NAMES = ('gemini-robotics-er-2-streaming-preview', 'gemini-3.1-flash-live-preview',
               'gemini-2.5-flash-native-audio-preview-12-2025')
TTS_MODELS = ('gemini-3.1-flash-tts-preview', 'gemini-2.5-flash-preview-tts')
VOICES = ('Puck', 'Charon', 'Kore', 'Fenrir', 'Zephyr')


def launch_config(model=CONFIG['modelName'], tts_model=CONFIG['ttsModelName'], voice=CONFIG['voiceName']):
    if model not in MODEL_NAMES or tts_model not in TTS_MODELS or voice not in VOICES:
        raise BootstrapError('Unsupported onboard model or voice selection.')
    return {**CONFIG, 'modelName': model, 'ttsModelName': tts_model, 'voiceName': voice}


def directory_id(directory: Path) -> str:
    return hashlib.sha256(str(directory.resolve()).encode()).hexdigest()


class BootstrapError(ValueError):
    def __init__(self, message, status=403):
        super().__init__(message)
        self.status = status


def loopback_origin(value: str) -> str:
    try:
        parsed = urlsplit(value)
        if (parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1', '::1')
                or parsed.username is not None or parsed.password is not None
                or parsed.path or parsed.query or parsed.fragment
                or not (1 <= (80 if parsed.port is None else parsed.port) <= 65535)
                or value != 'http://' + parsed.netloc):
            raise ValueError()
    except (ValueError, TypeError):
        raise BootstrapError('Onboard voice requires an exact HTTP loopback origin.') from None
    return value


def private_directory(path: Path) -> Path:
    # Do not chmod or follow an existing foreign/shared directory.
    if not path.is_absolute():
        raise BootstrapError('Onboard bootstrap directory must be absolute.')
    try:
        path.mkdir(mode=0o700, parents=False, exist_ok=True)
        info = path.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o700):
            raise BootstrapError('Onboard bootstrap directory must be owned and mode 0700.')
    except OSError:
        raise BootstrapError('Onboard bootstrap directory is unavailable.') from None
    return path


def read_private(path: Path, limit: int) -> bytes:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) & 0o077 or info.st_size > limit):
            raise BootstrapError('Onboard private file ownership, permissions or size is invalid.')
        with os.fdopen(fd, 'rb', closefd=False) as source:
            return source.read(limit + 1)
    finally:
        os.close(fd)


def grant_path(directory: Path, nonce: str) -> Path:
    if not isinstance(nonce, str) or not NONCE.fullmatch(nonce):
        raise BootstrapError('Invalid onboard launch grant.')
    return directory / (hashlib.sha256(nonce.encode()).hexdigest() + '.grant')


def issue_grant(directory: Path, origin: str, server_instance: str, *, config=None) -> tuple[str, Path]:
    private_directory(directory)
    loopback_origin(origin)
    if not isinstance(server_instance, str) or not re.fullmatch(r'[0-9a-f]{32}', server_instance):
        raise BootstrapError('Existing API identity is unavailable.')
    nonce = secrets.token_hex(32)
    path = grant_path(directory, nonce)
    now = time.monotonic()
    data = {'origin': origin, 'server_instance': server_instance, 'issued': now, 'expires': now + TTL_SECONDS, 'config': checked_config(config or CONFIG)}
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as output:
        json.dump(data, output)
    return nonce, path


def checked_config(config):
    if not isinstance(config, dict) or set(config) != set(CONFIG):
        raise BootstrapError('Invalid onboard configuration.')
    expected = launch_config(config['modelName'], config['ttsModelName'], config['voiceName'])
    if config != expected:
        raise BootstrapError('Invalid onboard configuration.')
    return dict(expected)


@contextmanager
def locked_grant(path: Path):
    fd = None
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # A second opener may have reached the inode just before consumption.
        if os.fstat(fd).st_nlink != 1:
            raise BootstrapError('Onboard launch grant was already consumed.', 410)
        yield
    except OSError:
        raise BootstrapError('Onboard launch grant unavailable; start a new explicit launch.', 410) from None
    finally:
        if fd is not None:
            os.close(fd)


class OnboardBootstrap:
    def __init__(self, directory: Path, key_file: Path):
        self.directory = private_directory(directory)
        if not key_file.is_absolute():
            raise BootstrapError('Onboard key file must be absolute.')
        self.key_file = key_file

    @classmethod
    def from_environment(cls):
        directory = os.environ.get('PUPPER_ONBOARD_BOOTSTRAP_DIR')
        if not directory:
            return None  # No generic key endpoint is enabled by default.
        key_file = os.environ.get('GEMINI_KEY_FILE')
        if not key_file:
            raise BootstrapError('Onboard voice requires GEMINI_KEY_FILE.')
        return cls(Path(directory), Path(key_file))

    @staticmethod
    def check_request(request) -> str:
        host = request.headers.getlist('host')
        origin = request.headers.getlist('origin')
        try:
            peer_ok = request.client is not None and ipaddress.ip_address(request.client.host).is_loopback
        except ValueError:
            peer_ok = False
        if (not peer_ok or len(host) != 1 or len(origin) != 1
                or any(name in request.headers for name in ('forwarded', 'x-forwarded-for', 'x-forwarded-host', 'x-forwarded-proto'))):
            raise BootstrapError('Onboard bootstrap requires a direct same-origin loopback request.')
        expected = loopback_origin('http://' + host[0])
        if origin[0] != expected:
            raise BootstrapError('Onboard bootstrap origin does not match its host.')
        return expected

    def consume(self, nonce: str, origin: str, server_instance: str) -> dict:
        path = grant_path(self.directory, nonce)
        with locked_grant(path):
            return self._consume_locked(path, origin, server_instance)

    def _consume_locked(self, path: Path, origin: str, server_instance: str) -> dict:
        try:
            data = json.loads(read_private(path, 2048))
            if not isinstance(data, dict):
                raise ValueError()
            config = checked_config(data.get('config'))
            now = time.monotonic()
            if (data.get('origin') != origin or data.get('server_instance') != server_instance
                    or not isinstance(data.get('issued'), (float, int))
                    or not isinstance(data.get('expires'), (float, int))
                    or not 0 <= now - data['issued'] <= TTL_SECONDS
                    or not 0 < data['expires'] - now <= TTL_SECONDS):
                raise BootstrapError('Onboard launch grant expired or belongs to another API.', 410)
            # Exclusive file lock spans validation, consumption and key reading.
            path.unlink()
        except (OSError, ValueError, TypeError) as error:
            if isinstance(error, BootstrapError):
                raise
            raise BootstrapError('Onboard launch grant unavailable; start a new explicit launch.', 410) from None
        try:
            key = read_private(self.key_file, 4096).decode().strip()
            if not key or any(character.isspace() for character in key):
                raise ValueError()
        except (OSError, ValueError):
            raise BootstrapError('Onboard credential is unavailable; check the configured private key file.', 503) from None
        return {'schema': SCHEMA, 'apiKey': key, 'config': config, 'autoConnect': True}
