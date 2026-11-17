# Pupper build and test handoff

Updated 2026-11-17 UTC (November 16 in California). Use the
[checkpoint](../foundation/results/interruption-recovery.json) for the current
operator, readiness and active run. This document explains the workflow; its
release records below are dated evidence, not permission to start the robot.

## Quick start: one owner, one selected release, one session

1. **Agree who owns the robot.** Exchange operator, boot ID, release/profile, run
   directory, manager/launch identities, camera/recorder owners and motor state.
   A teammate's active session remains theirs until an explicit handoff.
2. **Select a completed release and profile.** Shared Git source, a build directory
   and the mutable platform install are not release selectors. Never overwrite
   selected binaries/configuration or install into another person's live runtime.
3. **Check identity before hardware startup.** For a new session, use a fresh shell,
   the selected release environment and the preflight command below with the
   manager stopped. For an already homed matching session, retain its receipt and
   run the loaded check; do not prepare or home again for each trial.
4. **Start only with the supervisor ready and robot supported.** The selected
   launch **can home the legs**, even with the walker inactive. Verify the loaded
   runtime before the owner's deliberate arm/activation step. A staged release or
   passing test is not physical acceptance.
5. **Reuse that session for short trials.** Follow the
   [axis-pulse and headbutt runbook](behaviors/ROBOT_RUNBOOK.md). Stop the shot
   adapter before isolated pulses; do not add a second mux or teleop publisher.
   Save one receipt plus actual travel, braking, stability and contact observations.
6. **Close or hand over explicitly.** The owner stops the trial, supports the robot
   for driver shutdown, closes owned processes/recordings and records the result.
   Keep failed runs. An uncertain exit or changed boot needs identity recovery;
   do not restart blindly or kill unrelated processes.

## Ownership and directories

Shrish now owns movement, X/Y sprinting, the run-through/headbutt baseline,
gameplay integration and the arena. Tej owns foot kicking; Ryan owns overhead
perception and shooting geometry. Earlier movement assignments to Marcus are
historical. Physical get-up remains separate open work. Capability ownership does
not authorize starting processes in another person's robot session.

| Location | Purpose and boundary |
| --- | --- |
| `/home/pi/mts_123` | Shared Git source. Coordinate pulls/edits; source synchronization does not install or select a runtime. |
| `/home/pi/pupperv3-monorepo/ros2_ws/install` | Mutable legacy platform installation. No global rebuild/install/overwrite during an owned session. |
| `/home/pi/pupper-fc-native-build-20261116` | Preserved isolated Pi build. Use a new directory and one owner for subsequent builds. |
| `/home/pi/pupper-fc-releases/movement-20261116-vN` | Frozen release. New version for changed binaries, policy, launch or configuration; never overwrite a selected version. |
| `/home/pi/pupper-fc-runs/<unique-run>` | Receipts, logs, recordings and operator observations. Do not reuse a completed run directory. |
| `/home/pi/kick` | Recovered teammate experiment. Preserve its original files/logs. |

`pupper_gait_deploy/install.py` and `deploy.sh` target the platform installation;
`deploy.sh --no-launch` still installs and rebuilds. They are not staging checks.
Walk/sprint use separately pinned manifests. The manifest's `trial_profile` must
match the trial's `--profile`. Native parameters are cached at initialization;
changing their ROS parameter-store values does not establish applied settings.
A profile change needs an owner-coordinated fresh session.

## Build target and the repeated compatibility mistake

Read-only target check at **2026-11-17 02:08:18 UTC**:

| Component | Verified Pi value |
| --- | --- |
| OS / architecture | Debian 12 Bookworm / aarch64 |
| Compiler / C library | GCC `12.2.0-14+deb12u1` / glibc 2.36 |
| C++ runtime | `libstdc++.so.6.0.30`; provides through `GLIBCXX_3.4.30`, `CXXABI_1.3.13` |
| ROS underlay | `/opt/ros/jazzy` exists; complete dependency identity not yet frozen |

The cached `cs123-ros-jazzy-slew-check:latest` image is ARM64 **Ubuntu 24.04**,
for offline ROS tests only. v1 required `GLIBC_2.38` / `GLIBCXX_3.4.32` and was held
before activation. We repeated the known November 2 failure by packaging
Ubuntu test outputs for the older Pi. This was a release-process mistake.
Matching ARM64 and ROS Jazzy does not establish binary compatibility.

For now, build reviewed source in a separate Pi workspace against its existing
underlay; preserve the global installation. Keep source hashes, compiler identity,
test output and artifact hashes. Later, create an ARM64 Debian 12/GCC 12 builder
with the exact Pi ROS/dependency closure. No verified cached builder or complete
sysroot exists. Changing only the Ubuntu compiler or selecting an older ROS
distribution is insufficient.

Offline checks from the Mac checkout:

```sh
python3 foundation/scripts/session_status.py
.venv-api/bin/python -m pytest -q foundation/hardware/test_runtime_identity.py final_project/behaviors/tools/test_movement_trial.py
python3 foundation/scripts/check_ros_offline.py --output /absolute/new/evidence-directory
```

The last command tests inside the Ubuntu container; its outputs are not approved
Pi deployment binaries.

## Selected release and startup commands

For battery-swap/reboot checks, use a fresh bounded SSH connection:
`ssh -S none -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 mts`.
The Mac's November 16 `mts` configuration reused a connection for ten minutes
and allowed roughly two minutes of unanswered keepalives. A stale shared connection
can therefore delay recovery after power-off even with `ConnectTimeout` supplied.
This command avoids that connection without changing anyone's SSH configuration.

At **2026-11-17 02:10:56 UTC**, the build owner reported
`/home/pi/pupper-fc-releases/movement-20261116-v2` staged: Pi-native build/tests,
pinned files/config-policy link, four ROS package prefixes and target `ldd`
checks passed. No hardware startup or physical acceptance was included. v1 stays
held. Use a newer complete receipt if the owner supersedes this version.

| v2 manifest | Reviewed SHA256 |
| --- | --- |
| `walk-manifest.json` | `d21913f0f6addf332c7bac2ebebd89ea85637dda40d7a1c33d687e47c58c9bb2` |
| `sprint-manifest.json` | `54afc5e568b8bbf54a0068c269565fbe53937c40c6ac862fcdeb9901e103a7b0` |

The stock launch needs pinned **driver, controller, config, policy, launch, URDF,
gateway and mux** entries. Use the release's copied checker/trial tools. Fully
expand the configuration before preflight. Populate the following variables from
the reviewed release handoff; `PUPPER_MANIFEST` selects one profile. Use a fresh
shell and source only that release's environment, which sets its intended
underlay/overlay order; do not accumulate incidental overlays from older sessions.

```sh
source "$PUPPER_RELEASE/environment.sh"
export PYTHONDONTWRITEBYTECODE=1
mkdir "$PUPPER_RUN"  # New child of the existing /home/pi/pupper-fc-runs directory; stop on error.
python3 "$PUPPER_RELEASE/foundation/hardware/runtime_identity.py" preflight \
  --manifest "$PUPPER_MANIFEST" --sha256 "$PUPPER_MANIFEST_SHA256" \
  --output "$PUPPER_RUN/preflight.json"
```

Preflight starts no ROS/hardware. It checks pinned files, the configured policy,
absence of a manager, and target dependencies. Gateway/mux `ldd` coverage requires
them to be present in the manifest. Dependency refusal means fix the build, not
bypass the check. The launch separately verifies selected ROS package resolution.

Only after current supervisor readiness is established:

```sh
export PUPPER_RUNTIME_RECEIPT="$PUPPER_RUN/preflight.json"
export PUPPER_IMU_STARTUP_TRACE="$PUPPER_RUN/imu-startup.csv"
ros2 launch "$PUPPER_RELEASE/foundation/hardware/stock_session.launch.py"
```

**This can home.** Keep its owning terminal/session and logs. It starts one private
mux, stop gateway and inactive walker, retains joystick stop input, and does not
start teleop, camera or voice. From a second terminal with the same selected
environment, verify before the deliberate arm/activation step:

```sh
python3 "$PUPPER_RELEASE/foundation/hardware/runtime_identity.py" loaded \
  --receipt "$PUPPER_RUN/preflight.json"
```

The check binds boot, a newly started manager, supplied configuration bytes and
actual driver/controller mappings by path/inode/device. It does not inspect
weights in memory or prove physical behavior. v2 does not cover gateway/mux
running executable identity. Current source and the offline v3 package add that
check: exact executable path/device/inode, PID/start time, no duplicate or old
process, and no stopped/zombie process. Trials recheck these identities. This is
a process snapshot, not a lease or proof of application health. The [runbook](behaviors/ROBOT_RUNBOOK.md) supplies the exact
receipt-required pulse commands and explains command receipts versus measured
motion. Its stock layout is **20 frames × 48 values**, command indices **6:9** in
the newest frame.

The [offline continuation receipt](../foundation/results/movement-offline-followthrough-20261116.json)
identifies the v3 package and its new Python sources. Its native binaries and
stock actor are byte-identical to v2; no rebuild is needed. v3 is **local only**
until staged into its own new directory and checked on the target. Do not mix
its helper files with a v2 manifest or reuse the closed preflight after reboot.

## Shutdown, rollback and next improvements

A command zero, gateway stop and full driver shutdown have different effects.
The session owner closes the owned launch/recorder by recorded identity; do not
use a broad process kill. Record manager exit, remaining owned processes, closed
recording and the operator's physical observation before handing over. A software
exit alone does not prove motor release. See the
[hardware shutdown procedure](../foundation/HARDWARE_CHECKLIST.md).

Rollback selects a previously recorded **whole release/profile** after the old
session shuts down, then repeats preflight and supervised startup. Never replace
a live library or mix unrelated historical manifests. The R5 driver
`1dd92b65851bf9dae53aac28b3e20f6f81c70edadb28dd84dcbadcb9606ff429`
has historical physical evidence, not blanket acceptance of a new stack.

The local ABI screen now rejects missing target version definitions before
transfer. It reads ELF metadata; it never executes candidate code. From the
source checkout, pass the candidate binaries explicitly:

```sh
python3 foundation/scripts/check_pi_abi.py \
  --target foundation/results/build-environment-audit-20261116.json \
  --docker-image cs123-ros-jazzy-slew-check:latest \
  /absolute/candidate/libneural_controller.so /absolute/candidate/estop_controller
```

Local GNU `readelf` is used when available; otherwise the explicit cached Docker
image supplies only the metadata reader, with no network and read-only inputs.
Exit 0 means architecture/version requirements passed, 1 means a mismatch and
2 means inspection failed. This does **not** replace target `ldd`, dependency
closure or loaded-runtime verification. The target baseline pins provider hashes
and exact exported version names; refresh it after a target runtime upgrade.
ELF files without ordinary section tables (including sectionless ELF and extended
section numbering) are refused. The checker does not read dynamic-only version
tables; an empty `readelf` result must not be used to bypass that refusal.

Keep generated `__pycache__`/`.pyc` files out of future packages. v2's imported
checker regenerated one nonselected cache entry; the selected release identities
remain valid. Use the environment setting above to avoid changing cached files
during setup; do not edit or repackage v2 to hide that historical receipt detail.

Remaining small improvements:

1. Make the existing packaging path require this pre-transfer screen's receipt;
   retain target `ldd` checks.
2. Freeze the Pi build baseline and create the matching offline builder. Label
   test-only versus deployable outputs distinctly.
3. Validate v3's new executable checks and pulse reporting in the next selected
   session. Keep operator ownership explicit; process identity alone is not a
   distributed lock across teammates.

Evidence and remaining gaps: [build audit](../foundation/results/build-environment-audit-20261116.json).
Movement candidates: [offline comparison](../foundation/results/movement-readiness-20261116.json).
