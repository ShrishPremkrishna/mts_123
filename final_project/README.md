# Pupper FC

> **Current work (since Nov 23, 2026): [`fc/`](fc/README.md), a lean rebuild on branch `fc-v1`.** One laptop
> process (overhead camera → brain → signed UDP) drives each robot's stock walker through a small Pi
> bridge. Floor plan: [fc/RUNBOOK.md](fc/RUNBOOK.md); contract: [fc/SPEC.md](fc/SPEC.md). The previous FC
> stack is historical (see below); the Codex Phase 2–4 rebuild and other superseded `codex/*` branches are archived
> at `archive/*` tags. On Nov 24 the laptop drove `mts` on the floor (goto, push goals, fast gait); results and
> limits are in [fc/STATUS.md](fc/STATUS.md). Demo UI: Play screen at `/` (tests at `/ops`), launched with `fc/run.sh`.

## Start here

- [fc/STATUS.md](fc/STATUS.md): what works, how it was verified, what is next. Read first.
- [fc/DEMO.md](fc/DEMO.md): the demo plan (Shootout → Keeper → Switch → Match), gates, dates, run of show.
- [fc/RUNBOOK.md](fc/RUNBOOK.md): floor session steps.
- [fc/SPEC.md](fc/SPEC.md): the module and wire contract.

## Historical (pre-Nov 23)

The earlier FC stack (laptop lobby and replay, leg-kick executor, behaviors runbooks, Pi build/test handoff,
delivery plan, demo script, Nov 4 planning and workstream handoffs) was removed from the tree on 2026-11-24.
Everything is preserved at git tag `archive/pre-fc-v1-cleanup` (`git show archive/pre-fc-v1-cleanup:final_project/README.md`).
What remains: [overhead_perception](overhead_perception/README.md) (Ryan's camera pipeline, used by `fc/eye.py`) and the
[pupper_fc](pupper_fc/README.md) / [behaviors](behaviors/README.md) library remnant still imported by `gemini_pupper`'s `fc_walk_kick` mode.
