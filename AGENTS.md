# Pupper project context

[README.md](README.md) maps the code, setup and available resources. [The project brief](final_project/README.md) separates agreed outcomes from implementation status. Run `python3 foundation/scripts/session_status.py` for the dated task and access checkpoint. The user's current request sets the task. Earlier plans remain available as historical references.

**Current FC work (since Nov 23, 2026) is [`final_project/fc/`](final_project/fc/STATUS.md), branch `fc-v1`.** Lean v1: one laptop process (camera → brain → signed UDP streaming velocity at 30 Hz) and a small Pi bridge into the stock walker. Start with [fc/STATUS.md](final_project/fc/STATUS.md) and [fc/DEMO.md](final_project/fc/DEMO.md). Physical tests run as UI/`fcctl` pipelines ([RUNBOOK](final_project/fc/RUNBOOK.md)) that log themselves; the operator runs them. Older stacks under `final_project/` are historical.

## Working approach

- Infer a useful role from the current request, source evidence and team context. Team ownership guides coordination, not the scope of help you can offer; handoffs are optional working notes.
- State consequential assumptions and define an observable result. Resolve reversible details with judgment and ask when the answer changes the goal or required access.
- Prefer understandable solutions sized to the task. Existing architecture is a starting point and may change when evidence or the goal warrants it.
- **Speed.** Shorten time to a verified outcome. Fix recurring tooling friction and parallelize useful work with Claude Code or focused subagents. Delegate outcomes, relevant context, constraints and success evidence; leave methods open and invite better alternatives. Give concurrent editors clear file ownership; subagents edit only the files they were given and `git add` only those.
- **Altitude.** Reassess the outcome behind the immediate task. Challenge premises constructively and propose a better direction when evidence or the user's goal warrants it.
- **Depth.** Examine source data and underlying mechanisms. Use appropriate instruments and independent checks to challenge assumptions and verify consequential claims.
- **Range.** Draw on relevant disciplines and compare promising alternatives through concrete experiments. Keep exploratory ideas distinguishable from implemented behavior.

These principles adapt the [Karpathy-inspired coding guide](https://github.com/multica-ai/andrej-karpathy-skills/blob/main/CLAUDE.md) and [Rieseberg's ambition framework](https://felixrieseberg.com/how-to-be-more-ambitious-a-tutorial-for-humans-with-models/).

## Keep the project ready for the next session

- Update [fc/STATUS.md](final_project/fc/STATUS.md) when a capability changes (what works, how verified: sim / device-free / robot). Routine changes need no doc edit. Replace superseded claims; distinguish observed facts from assumptions. Prefer few, short, current documents.
- Keep durable principles in `AGENTS.md`, current status in STATUS.md, the plan in DEMO.md, setup/interfaces near their code, and active task/access state in the [checkpoint](foundation/results/interruption-recovery.json). Link to authoritative facts instead of repeating them. Git history preserves routine revisions; do not write evidence archives or per-run reports into the repo.
- Keep the environment reproducible as it evolves. When dependencies, commands or deployment behavior change, update the relevant pins and setup instructions and verify the affected path. Repair demonstrated friction; avoid gratuitous upgrades or frameworks. Date machine-specific observations and recheck them when relevant.
- Inspect branch and working changes before edits; preserve others' work. Use descriptive branches when experimentation or parallel work benefits from isolation. Review your diff and make coherent commits to retain recoverable progress. Use history to investigate regressions and recover working versions. Coordinate integration into shared `main` and publishing within the user's authorization, consulting the session checkpoint for current team/access constraints. Do not overwrite concurrent work or rewrite shared history without agreement.

## Access and physical effects

`main` is the shared integration branch. Package-specific agent files explain local interfaces and commands. Their sibling `CLAUDE.md` files import the same guidance.

Mac environments, setup and verification commands are in the README. Keep credentials external to source, logs and shared artifacts.

Robot sessions have one operator coordinating camera, motion and services. Check current ownership and readiness before hardware operations. In fc, `fcctl ready --confirm` (the UI's Ready) can home the legs: robot on the stand. Command effects and stops are in [fc/robot/README.md](final_project/fc/robot/README.md) and the [RUNBOOK](final_project/fc/RUNBOOK.md); the older [hardware procedure](foundation/HARDWARE_CHECKLIST.md) covers the old stack. Simulate changed motion before supervised use and obtain applicable course approval for new policies. Preserve joint/current limits unless an explicitly reviewed change warrants different values.

Distinguish source checks, staged files, controller receipts and observed physical outcomes. Identify the actual running release when recording hardware evidence.
