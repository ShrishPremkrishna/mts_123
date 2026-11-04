# Pupper working agreement

`main` is the shared branch. The [README](README.md) maps the repository. `gemini_pupper/` holds Lab 6 and the general harness. `final_project/` holds soccer-specific work. `foundation/` holds releases and evidence.

## Start here

Run `python3 foundation/scripts/session_status.py`. Add `--verify-inputs` to check the pinned local inputs and API interpreter. [The current-state record](foundation/results/interruption-recovery.json) is canonical. Dated reports are historical evidence only. Use `.venv-api/bin/python` for course and API checks. `.venv/bin/python` exists only in a provisioned numerical checkout.

## Editing

Adapted from the community’s [Karpathy-inspired guidelines](https://github.com/forrestchang/andrej-karpathy-skills/blob/main/CLAUDE.md).

- State material assumptions and how you will verify success.
- Prefer the simplest complete change using existing tools.
- Make surgical edits. One agent owns each changed file, and the lead integrates and commits.
- Verify the normal path. Also verify failures and cancellation when they apply.

## Robot and evidence

- One supervised operator owns camera, motion and service restarts for each block. Follow the [hardware checklist](foundation/HARDWARE_CHECKLIST.md).
- The maintained `start-native` step initializes motors and can home the legs. Confirm operator readiness before starting it or commanding motion.
- Offline work never moves the robot. Document proposed limit or gain changes in a separate profile and simulate them before supervised use.
- Simulate new or changed motion. New policies need the applicable approval before they first run on hardware.
- Verify the running release before attributing a physical observation to it. Record software checks, staging and observed motion separately.
- Preserve the exact source, configuration, policy and outcomes. Long jobs need a verified checkpoint and a verified final archive.
- Keep credentials out of source, logs and shared artifacts.
- When operational state changes, update the current-state record for handoff.
