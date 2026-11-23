"""Explicit, browser-only onboard voice; attaches to an already healthy API.

Enable PUPPER_ONBOARD_BOOTSTRAP_DIR and GEMINI_KEY_FILE in that API's environment.
Then run: python3 robot/kiosk.py --url http://localhost:18090 --bootstrap-dir /run/user/1000/pupper-onboard
No ROS, API or motor startup, CDP, reconnection, key injection or device selection.
USB microphone and speaker must already be selected as the OS default devices.

--stop-on-browser-exit (off by default) is for exclusive onboard sessions only.
After the owned browser returns, for any exit status, or when this launcher is
interrupted by SIGINT/SIGTERM, it sends one global Stop to the API instance it
launched against. That Stop also stops every other client's activity on that
API. Older APIs without the dedicated instance-guarded Stop are refused before
launch; if the API is rolled back during the session, Stop is unconfirmed and
never falls back to the unrestricted operator endpoint. It is not a release, homing or shutdown and does not prove the robot is
physically still. On an interrupt subprocess.run kills (SIGKILL) and reaps
the direct browser process; this is not a graceful browser exit. Not covered:
SIGKILL of this launcher, a hung renderer whose browser never exits, and host
or API failure. Exits nonzero (keeping a nonzero browser status) unless the
original API confirms that Stop.
"""
from __future__ import annotations

import argparse
import fcntl
import html
import http.client
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import urllib.request

from onboard_bootstrap import BootstrapError, CONFIG, MODEL_NAMES, TTS_MODELS, VOICES, SCHEMA, TTL_SECONDS, directory_id, issue_grant, launch_config, loopback_origin, private_directory

STOP_INSTANCE_HEADER = 'X-Pupper-Expected-Server-Instance'
OWNED_STOP_PATH = '/stop/owned'
STOP_UNCONFIRMED = ('Onboard browser session ended, but Stop of the original API was not confirmed. '
                    'Check the robot and use the operator Stop.')


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *_args, **_kwargs):
        raise BootstrapError('Onboard API redirects are not allowed.')


def get_json(origin, path):
    request = urllib.request.Request(origin + path, headers={'Origin': origin})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    with opener.open(request, timeout=3) as response:
        return json.loads(response.read(8192))


def post_stop(origin, server_instance) -> bool:
    """One Stop attempt, no retry; True only for the original instance's receipt."""
    # A distinct route is essential: old servers ignore unknown headers on
    # ordinary Stop. They reject this route without stopping a replacement API.
    request = urllib.request.Request(origin + OWNED_STOP_PATH, data=b'', method='POST',
                                     headers={'Origin': origin, STOP_INSTANCE_HEADER: server_instance})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    try:
        # The timeout bounds each socket operation, not the whole exchange.
        with opener.open(request, timeout=3) as response:
            body = response.read(8193)
        document = json.loads(body) if len(body) <= 8192 else None
    except (OSError, ValueError, RecursionError, http.client.HTTPException):
        return False  # Includes 409 not_dispatched, redirects, timeouts and malformed JSON.
    return (isinstance(document, dict) and document.get('success') is True
            and document.get('server_instance') == server_instance
            and document.get('command_epoch_required') is True
            and isinstance(document.get('command_epoch'), str) and document['command_epoch'] != '')


def launch(origin: str, directory: Path, *, browser='chromium', get=get_json, run=subprocess.run, model=CONFIG['modelName'], tts_model=CONFIG['ttsModelName'], voice=CONFIG['voiceName'],
           stop_on_browser_exit=False, stop=post_stop) -> int:
    origin = loopback_origin(origin)
    config = launch_config(model, tts_model, voice)
    directory = private_directory(directory)
    # One explicit voice browser per bootstrap directory, held for its lifetime.
    fd = os.open(directory / 'browser.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise BootstrapError('An onboard browser launcher is already active.') from None
        health = get(origin, '/health')
        if health.get('status') != 'running' or health.get('tools') is not True:
            raise BootstrapError('The selected API is not healthy; no browser was launched.')
        status = get(origin, '/onboard/status')
        if status.get('schema') != SCHEMA or status.get('enabled') is not True or status.get('directory_id') != directory_id(directory):
            raise BootstrapError('The selected API has not enabled onboard bootstrap.')
        if stop_on_browser_exit and status.get('instance_guarded_stop') is not True:
            raise BootstrapError('The selected API does not support instance-guarded browser-exit Stop; no browser was launched.')
        profile = directory / 'browser-profile'
        if (profile / 'SingletonLock').exists() or (profile / 'SingletonLock').is_symlink():
            raise BootstrapError('The onboard browser profile is already owned; close it before starting another.')
        instance = status.get('server_instance')
        nonce, grant = issue_grant(directory, origin, instance, config=config)
        # Do not put the bearer nonce in Chromium argv, which another OS user
        # may inspect. Only this UID can read the short-lived local launch page.
        page_fd, page_name = tempfile.mkstemp(prefix='launch-', suffix='.html', dir=directory)
        launch_page = Path(page_name)
        with os.fdopen(page_fd, 'w') as page:
            target = html.escape(origin + '/#onboard=' + nonce, quote=True)
            page.write('<!doctype html><meta name="referrer" content="no-referrer">'
                       '<meta http-equiv="refresh" content="0;url=' + target + '">')
        def cleanup():
            for path in (launch_page, grant):
                try: path.unlink(missing_ok=True)
                except OSError: pass  # The grant still expires; never expose its contents.
        expiry = threading.Timer(TTL_SECONDS, cleanup)
        expiry.daemon = True
        expiry.start()
        def stop_after_exit():
            # The browser has returned: drop its grants, then Stop while the lock is held.
            expiry.cancel()
            cleanup()
            confirmed = False
            try:
                confirmed = stop(origin, instance) is True
            except Exception:
                pass  # Unconfirmed; never echo API or callback details.
            finally:
                if not confirmed:
                    print(STOP_UNCONFIRMED, file=sys.stderr)
            return confirmed
        try:
            # Profile and media grants are local to this one owned browser.
            command = [browser, '--ozone-platform=wayland', '--kiosk', '--noerrdialogs',
                       '--disable-infobars', '--no-first-run', '--no-default-browser-check',
                       '--password-store=basic', '--use-fake-ui-for-media-stream',
                       '--autoplay-policy=no-user-gesture-required',
                       f'--user-data-dir={directory / "browser-profile"}', launch_page.as_uri()]
            # Keep the lock in Chromium as well: a launcher crash cannot admit a
            # second voice browser while the first process still owns the profile.
            # A spawn OSError propagates without Stop: no browser session existed.
            try:
                result = run(command, pass_fds=(fd,), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except (KeyboardInterrupt, SystemExit):
                # subprocess.run has already killed and reaped its direct child.
                if stop_on_browser_exit:
                    try: stop_after_exit()
                    except BaseException: pass  # Never replace the original interrupt.
                raise
            if stop_on_browser_exit and not stop_after_exit():
                return result.returncode or 1  # An unconfirmed Stop is never reported as success.
            return result.returncode
        finally:
            expiry.cancel()
            cleanup()
    finally:
        os.close(fd)


def terminate(signum, _frame):
    # Unwind like Ctrl-C so subprocess.run reaps its own browser before Stop.
    raise SystemExit(128 + signum)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True, help='Existing HTTP loopback API origin (no path).')
    parser.add_argument('--bootstrap-dir', required=True, type=Path)
    parser.add_argument('--model', choices=MODEL_NAMES, default=CONFIG['modelName'])
    parser.add_argument('--tts-model', choices=TTS_MODELS, default=CONFIG['ttsModelName'])
    parser.add_argument('--voice', choices=VOICES, default=CONFIG['voiceName'])
    parser.add_argument('--stop-on-browser-exit', action='store_true',
                        help='Exclusive onboard sessions only: one global Stop of the original API when the browser session ends.')
    args = parser.parse_args(argv)
    # Only the opt-in changes SIGTERM; default signal semantics are unchanged.
    previous = signal.signal(signal.SIGTERM, terminate) if args.stop_on_browser_exit else None
    try:
        return launch(args.url, args.bootstrap_dir, model=args.model, tts_model=args.tts_model, voice=args.voice,
                      stop_on_browser_exit=args.stop_on_browser_exit)
    except (BootstrapError, OSError, ValueError):
        # Do not reflect process argv, launch nonce, HTTP bodies or credentials.
        print('Onboard launch failed. Check the enabled API, private bootstrap directory, and existing browser ownership.', file=sys.stderr)
        return 1
    finally:
        if args.stop_on_browser_exit:
            signal.signal(signal.SIGTERM, signal.SIG_DFL if previous is None else previous)


if __name__ == '__main__':
    raise SystemExit(main())
