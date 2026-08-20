---
name: doctor
description: Health-check a project's agent-readiness. Scores the repo against the blueprint's checks, lists concrete gaps, and flags drift (broken commands in CLAUDE.md, stale context, stale worktrees). Use on demand after install, before scaling up delegation, or when agents start failing at tasks they used to handle.
---

# doctor — agent-readiness health check

> Status: v0 draft — check list to be encoded as executable checks during implementation.

## Checks (v0 set)

**Context**
- `CLAUDE.md`/`AGENTS.md` exists, is under ~150 lines, contains always/ask-first/never boundaries
- No generated/copied artifact is gitignored (`git check-ignore` CLAUDE.md, AGENTS.md, .claude/, HOW-WE-WORK-WITH-AI.md)
- Every command in the context file actually runs, copy-paste (execute them); flag commands that fetch tools over the network on first use (`npx` without a matching devDependency); execute `UNVERIFIED`-marked team-tool commands and remove the marker on success
- Context freshness: referenced paths exist; stack/version claims match reality
- Monorepo: per-package context files present where packages diverge

**Feedback loops**
- File-scoped lint and test commands exist and return in seconds
- One-command working dev environment (bootstrap/devcontainer)

**Guardrails**
- `.claude/settings.json` baseline present; protected paths from the install interview still enforced
- Hooks installed and firing (secret scan, protected paths)

**Safety net**
- Test suite exists and runs; critical paths (entry points, data-mutating code) are covered
- For coverage gaps, phrase the fix as: run `/write-tests <area>`

**Worktrees**
- No stale worktrees (merged or abandoned); conventions section present in context file

**Delegation health**
- Tracker/VCS CLI wired and authenticated
- `HOW-WE-WORK-WITH-AI.md` present and consistent with actual settings

## Output

A short scored report (per area: OK / gap / broken) with a concrete fix for every non-OK item — each fix phrased so it can be delegated to an agent directly. If everything is fine, say so in one line.
