---
name: workon
description: Take a ticket to a merged-ready PR, end-to-end. Usage - /workon <ticket-id>. Fetches the (ideally already triaged) ticket, always writes a plan document under docs/plans/ first — this is where business requirements get mapped to technical implementation — then creates a dedicated worktree, implements with tests, self-reviews, and opens a PR linked to the ticket. Asks the user about every unknown instead of guessing. Use for delegated implementation work.
---

# workon — ticket to PR

> Status: v0 draft.

## Scope rule (the contract with triage)

Triage delivers business-level tickets (what and why, no technical detail). Workon owns the **how**: the plan document is where business requirements are mapped to files, approach, and test strategy. Don't expect technical pointers in the ticket — deriving them is this skill's first job.

## Flow

1. **Fetch** the ticket via the team's tracker CLI (from `CLAUDE.md`). If it's vague or missing acceptance criteria, recommend running `/triage` first — or ask the clarifying questions inline for small tickets.
2. **Plan** (always): write a plan document to `docs/plans/<ticket-id>.md` — the technical mapping of the ticket: approach, files to touch, test strategy, risks. Tiny tickets get a tiny plan; no ticket skips it. If the files to touch lack the tests needed to catch regressions from this change, state that in the plan and recommend running `/write-tests <area>` first. The plan doubles as the PR's explanation of approach. Ask the user to confirm the plan when the ticket is large, risky, or touches ask-first paths; otherwise proceed and include the plan in the PR.
3. **Worktree**: create a dedicated worktree per the project's conventions (see `parallel-work`); never implement in a working copy another agent or human is using.
4. **Implement** with tests, respecting the context file's boundaries (always/ask-first/never). Run the file-scoped feedback loops continuously; run the full test suite before opening the PR.
5. **Self-review**: run `/review` on the branch before pushing. Fix what it finds.
6. **PR**: open it linked to the ticket, containing: what changed and why, evidence (test output, commands run), new dependencies flagged, boundary decisions made, the plan doc, and open questions if any.

## Rules

- Every unknown is a question to the user — same discipline as `triage`.
- Never push directly to the default branch.
- Ask-first paths require an explicit user go-ahead before touching them.
- If the task turns out bigger than the ticket implies, stop and say so — don't silently expand scope.
- Update the ticket's status/comments as you go so the tracker reflects reality.
