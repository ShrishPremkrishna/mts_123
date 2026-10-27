# pupper-rebase

Brings a class Pupper (the image flashed for CS 123 students) up to the
**cs123-build**, the stable base the Gemini Pupper lab is developed on.

`cs123-build` (commit `0e50afb`) = Nate711/pupperv3-monorepo `main` (6f96c5e) merged with
Ankush's `wifi-pairing-mode` (64c86d7). The class image sits at `main` b5b8c89 (2025-10-18).

## Use

```bash
./rebase.sh pupper --dry-run     # preflight + print every command, change nothing
./rebase.sh pupper               # apply, then verify and save manifests/pupper-after
```

`pupper` is any SSH host for the robot (passwordless SSH and sudo required).
Add `--skip-wifi-pairing` to leave WiFi management alone.

Or run it on the Pupper itself, with no SSH setup from a laptop (what students do in Lab 2):

```bash
git clone https://github.com/cs123-stanford/pupper_rebase.git && cd pupper_rebase
./rebase_local.sh --dry-run
./rebase_local.sh
```

`rebase_local.sh` runs the same steps and checks in the background and logs to
`~/cs123-rebase.log`, so a dropped SSH session doesn't stop it (reconnect and `tail -f` the log).
It ends with `REBASE OK` or `REBASE FAILED`. It doesn't save a manifest; run
`capture_manifest.sh` from a PC for that.

## What it does

| Step | Change |
|---|---|
| 1 Preflight | Monorepo must be at b5b8c89 or already at cs123-build, with only expected local changes; needs sudo and internet |
| 2 Keys | `lib/wipe_keys_remote.sh` blanks every key, secret and token in real env files under /home/pi |
| 3 Monorepo | Fetches cs123-build from the shipped git bundle and checks it out (LFS bag files are skipped) |
| 4 Packages | apt `libzbar0`; pip `fastapi uvicorn python-multipart pyzbar viser yourdfpy`, pinned |
| 5 ROS | Rebuilds only `animation_controller_py` (13 tricks) and `hailo` (QR pairing) |
| 6 Screen app | Installs the cross-compiled `pupper-rs`, enables and restarts `pupper-gui` |
| 7 Tailscale | Installs it, logged out; students run `sudo tailscale up` with their own account |
| 8 Claude Code | Installs it for user pi, logged out; students sign in on first `claude` |
| 9 WiFi pairing | Runs Ankush's comitup installer detached; hotspot `Pupper-Setup-<nnn>` |

Not installed: Nathan's LiveKit voice agent and its keys. The neural controller is left to
pupper_gait_deploy.

## Other tools

| Script | Purpose |
|---|---|
| `prepare.sh` | Rebuilds `artifacts/` on this PC: the git bundle and the aarch64 screen-app binary |
| `wipe_api_keys.sh <host> [--dry-run] [--include-demo-key]` | Key wipe on its own; `--include-demo-key` also removes the Gemini demo key and kiosk profile |
| `capture_manifest.sh <host> <name>` | Records packages, ROS versions, services and monorepo state into `manifests/<name>` |

`COMPARISON.md` has the old-Pupper vs class-image comparison and the decisions behind this build.

## Notes

- The WiFi-pairing hotspot password is set in the monorepo's `infra/wifi-pairing/comitup.conf` and is the same on every Pupper.
- `cs123-build` exists only in the bundle and in `~/projects/pupperv3-monorepo-upstream`; push it to a course fork if the Puppers should `git pull` it later.
