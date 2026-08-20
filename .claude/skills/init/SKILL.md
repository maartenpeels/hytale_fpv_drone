---
name: init
description: Make this project AI-native. Scans the repo, interviews the team about what a scan can't know, generates tailored context files, guardrails, and feedback-loop commands, verifies the result with a cold-agent self-test, and opens a single install PR. Use when a project adopts the Merapar AI-native blueprint for the first time — normally invoked by the blueprint's INSTALL.md right after it copies the skills into the repo.
---

# init — the installer

> Status: v0 draft — flow agreed, generation rules to be hardened against the eval repos.

## What you produce

A single branch + PR ("Make <project> AI-native (blueprint v<version>)") containing:

- The blueprint skills that INSTALL.md copied into this repo (commit them on this branch — the install PR carries the whole integration)
- `CLAUDE.md` (generated from the scan + interview, from `templates/CLAUDE.template.md`) and `AGENTS.md` as a symlink to `CLAUDE.md` — never a copy; if symlinks won't survive the team's platform or tooling, ask before falling back to a one-line pointer file
- `.claude/settings.json` permission baseline (from `templates/settings.template.json`, extended with interview answers)
- Hooks: secret scanning, protected-path enforcement. Write into the repo's existing hook mechanism (husky, lefthook, pre-commit, …); never overwrite existing hooks — extend them. If the repo has NO hook mechanism, do not silently write to `.git/hooks/` (it can't be committed, so the PR wouldn't carry it) — ask the team: adopt a hook framework, or generate a committed `scripts/` hook plus a documented one-line setup command; flag the choice in the PR. A committed content-scanning hook MUST exclude its own path from the scan (its own patterns otherwise block the install commit)
- File-scoped feedback-loop commands (lint one file, test one file) wired into the repo's existing task runner (make/npm/just/gradle — match what the repo uses)
- Per-package context files when the repo is a monorepo (nearest-file-wins)
- `HOW-WE-WORK-WITH-AI.md` (from `templates/HOW-WE-WORK-WITH-AI.template.md`)
- Worktree conventions section in `CLAUDE.md` (location, naming, one agent per worktree, cleanup)

## Flow

1. **Scan** the repo: language(s), frameworks, build/test/lint commands (verify each one actually runs — never write a command you haven't executed), directory structure, existing docs/context files, CI setup, git remote (detect GitHub/GitLab/Bitbucket), monorepo layout.
2. **Interview** the team — ask ONLY what the scan cannot know. If the installer (INSTALL.md) already established a fact this step needs (agent tooling, existing context/config files, blueprint version), use it — do not re-ask:
   - Protected paths (auth, payments, crypto, generated code) → always/ask-first/never boundaries
   - Client constraints on AI tooling (allowed tools, data flows, forbidden areas)
   - Which issue tracker the team uses and the CLI/MCP + auth choice for it
   - On a repo with no code yet (nothing to scan): the intended stack, package manager, test runner, and package-manifest conventions (license, private/public, version scheme) — the interview carries what the scan can't
3. **Generate** all artifacts. Context-file quality rules: keep it under ~150 lines; every command copy-paste runnable and verified — a command that doesn't exist (no build step, no lint setup) is OMITTED with a one-line note, never invented; commands that fetch tools over the network on first use (`npx <not-a-devDependency>`) get flagged as such; one real code example from this repo, quoted VERBATIM (no `// ...` elisions — an abridged example that differs from real behavior misleads agents); point at one exemplary test file for test conventions; explicit always/ask-first/never lists; wire the team's tracker/VCS CLI commands in — commands that cannot be executed at install time (tool not installed / not authenticated) are marked `UNVERIFIED: verify on first use`, never presented as verified.
4. **Check nothing is silently ignored**: run `git check-ignore` on every file you generated or copied (target repos are known to gitignore `CLAUDE.md` and similar AI files). Any hit → ask the team: un-ignore, rename, or keep local-only. Never let the PR ship without its artifacts because `git add` skipped them.
5. **Self-test (cold agent test)**: in a fresh context, take a trivial real task and verify the generated context is enough to build, test, and lint unaided. On a repo with no code yet, the task is a first feature (create the initial module + test using the team's declared stack). Every failure becomes a TODO in the PR description — do not silently fix and forget.
6. **Open the PR** with a summary of what was generated, the self-test result, and remaining TODOs.

## Rules

- Never overwrite files the team already has (existing CLAUDE.md, hooks, settings): merge, and mark the diff clearly in the PR.
- Every unknown is a question to the user, never a guess.
- Everything you generate is team-owned afterwards; mark managed sections explicitly.
- No CI workflows in v0 (that is the v0.1 CI layer).
