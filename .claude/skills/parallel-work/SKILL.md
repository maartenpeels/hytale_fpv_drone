---
name: parallel-work
description: Safe git-worktree management for parallel agent sessions. Creates a named worktree per task, enforces one-agent-one-worktree, lists active worktrees, and cleans up merged or stale ones. Use when starting delegated work (workon calls this), when running multiple agent sessions on one repo, or to clean up after merges.
---

# parallel-work — worktree management

> Status: v0 draft.

## Conventions (written into each project's CLAUDE.md by init)

- Worktrees live in the project's configured location (default: `.worktrees/<task-id>`, gitignored; some teams prefer `../<repo>-worktrees/`)
- One worktree per task, one agent per worktree — never share a working copy
- Branch naming: `<type>/<ticket-id>-<slug>` (matches the team's existing convention if one exists)
- Cleanup after merge is part of finishing the task, not optional

## Operations

- **create `<task-id>`**: verify no existing worktree for this task, create worktree + branch from up-to-date default branch, run the project's bootstrap so the environment works, report the path
- **list**: active worktrees with branch, age, last commit, and merge status
- **cleanup**: remove worktrees whose branches are merged; list stale ones (no commits > configurable days) and ask before removing those
- **guard**: before any operation, detect and warn if two sessions appear to share one worktree

## Rules

- Never remove a worktree with uncommitted changes without explicit user confirmation.
- Never create a worktree from a stale default branch — fetch first.
- Report bootstrap failures at create time immediately.
