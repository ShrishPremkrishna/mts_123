# Old Pupper vs class image

Captured 2026-09-26 with `capture_manifest.sh` (raw data in `manifests/`).
"Class" is the image flashed on the students' Puppers; the test Pupper now runs it too.
"Old" is JC's former development Pupper, snapshotted to `~/projects/pupper-test-snapshot`.
The raw manifests keep their capture names: `student-image` is the class image, `test-image` the old Pupper.

## Summary

The two images share the same base OS, kernel, ROS build and HailoRT. The old
Pupper is the class image plus later monorepo commits, a few apt upgrades,
and a voice-agent Python stack. Nothing on the class image is newer.

| Layer | Class | Old | Rebase action |
|---|---|---|---|
| Base OS | Raspberry Pi OS 2024-11-19, Debian 12 | Same | None |
| Kernel | 6.12.47+rpt-rpi-2712 | Same | None |
| ROS Jazzy, 426 packages | Source build | Identical versions | None |
| HailoRT | 4.20.0 | Same | None |
| apt packages | Baseline | 41 lines differ: minor upgrades plus comitup, python3-zbar, python3-networkmanager, tailscale, gh, iptables | Add only what the target monorepo needs |
| Python (pip) | 375 packages | 449: adds LiveKit, google-genai, OpenAI, viser, pyzbar and upgrades | Add only what the lab needs |
| Monorepo | main @ b5b8c89, 2025-10-18 | wifi-pairing-mode @ 64c86d7, 2026-07-20 | Fast-forward: student is a strict ancestor, 20 commits behind |
| Enabled services | No pupper-gui, comitup or tailscale | pupper-gui, comitup, tailscaled | Decide per service |
| Home folder | Only the monorepo | Course lab repos, pupper_gait_deploy, gait tuner | Labs are cloned by students per lab |

## Monorepo: the 20 commits the class image lacks

- **Tricks.** superman, pee and upward_dog recordings and CSVs; pushup and other problematic tricks removed from the agent's list. The class image has 10 animation CSVs, the old Pupper 13.
- **Voice agent.** Localhost mode, mode check, follow deactivation on stop, prompt personalization, emotion tags, latency tweaks.
- **Hailo detector.** QR-code WiFi pairing added to `hailo_detection.py`; it now imports `pyzbar`, which the class image lacks.
- **WiFi pairing.** comitup config, installer, NetworkManager patch, sudoers entry, GUI pairing button.
- **pupper-rs GUI.** QR scanning and pairing screens; needs a Rust rebuild. The class image has no Rust or Node toolchain.
- **robot.service / robot.sh / start_all_services.sh.** Service start-order changes.

## Old-Pupper-only local changes (uncommitted)

| Change | Carry over? |
|---|---|
| `neural_controller/*` | No: pupper_gait_deploy installs its own controller before the Gemini lab |
| `ros_tool_server.py`, `pupster.py`: greet patrol | Decide; the Gemini lab may replace this layer |
| `system_prompt.md`: GirlCon event persona | No |
| `gemini_local_server.py`: Peng's server copy | No; belongs to the lab repo |
| `enable_all_services.sh`, `disable_all_services.sh` | Optional helpers |
| `ros2_ws/src/common/` | Already identical on both images |
| hailort `.deb` files marked modified | Git LFS artefact; identical on both |

## API keys

| Where | What | Action |
|---|---|---|
| Class image files | No Google or other real keys; only `.env.example` templates. Confirmed by a `wipe_api_keys.sh --dry-run` | None |
| `/etc/chromium.d/apikeys` | Debian's own Chromium keys from the chromium package | None |
| Old Pupper `agent-starter-python/.env.local` | Nathan's real Google, OpenAI, LiveKit, Deepgram and Cartesia keys. Git-ignored, not in the repo, absent from the class image | Never copy. `wipe_api_keys.sh` blanks them on any Pupper before rebasing. The PC snapshot also holds a copy |
| Monorepo git history | A Gemini key committed by Nathan in e88ca6a (2025-08-19), removed in 3c30b84 (2025-09-01, "revoked key") | Also public on GitHub. Confirm revocation with Nathan; rewriting history is optional |

## What the Gemini lab needs on the class image

- Python: `fastapi`, `uvicorn`, `python-multipart`. Present already: `zmq`, `psutil`, `dotenv`, `PIL`.
- `ros_tool_server.py` imports `pupster.py`, which imports LiveKit. Either install the LiveKit stack or decouple the tool server so it doesn't need it.
- The lab launch file and config from `~/gemini_eval`, since the course's deploy script replaces the monorepo launch file.
- `pyzbar` and `python3-zbar` if the monorepo is fast-forwarded, because the new detector imports it.

## Decisions (2026-09-26)

- Target build `cs123-build` = upstream main 6f96c5e + wifi-pairing-mode 64c86d7, merged without conflicts into commit 0e50afb.
- Nathan's voice agent (LiveKit, OpenAI, Deepgram, Cartesia, llm-agent service) is not installed or enabled; students do embodied reasoning only through the Gemini Pupper lab.
- Included: WiFi pairing (comitup), the screen app (pupper-gui), Tailscale installed but logged out (the old Pupper's Tailscale belonged to Ankush's account), and Claude Code installed but logged out.
- Applied to the test Pupper on 2026-09-26; all 11 checks passed, including after a reboot.
