---
name: review
description: Adversarial code review with evidence requirements. Usage - /review <pr-number>, /review <branch>, or /review with no argument to review the working tree. Small diffs get a single-pass review; large diffs fan out into parallel per-dimension subagents (correctness, security, tests, blast radius) plus a mechanical boundaries check, with high-severity findings adversarially verified before reporting. Use for formal PR review and as a pre-push self-check.
---

# review — adversarial review

> Status: v0 draft.

## Scope resolution

- `<pr-number>` → fetch the PR diff via the team's VCS CLI (from `CLAUDE.md`)
- `<branch>` → diff against the default branch
- no argument → review the working tree (staged + unstaged)

## Architecture

**Step 0 — mechanical boundaries check (always, first).** Deterministic, no judgment involved: compare the diff's touched paths against the context file's always/ask-first/never lists. A `never` hit is an automatic blocker; an `ask-first` hit without a recorded user go-ahead is a blocker until confirmed.

**Step 1 — pick the mode by diff size.**

- **Small diff** (roughly < 200 changed lines): single-pass review covering all dimensions yourself.
- **Large diff**: fan out one subagent per judgment dimension, in parallel, each with fresh context, the diff, the intent (ticket/PR description/plan doc), and exactly one lens:
  - **Correctness** — trace the failure scenario before claiming a bug; concrete inputs → wrong output. No "this looks suspicious" without a trace.
  - **Security** — secrets, injection, authz gaps; flag every new dependency.
  - **Tests** — do they exist, do they test the change (not just pass), what's uncovered? Run them.
  - **Blast radius** — callers, migrations, API/config compat, rollout implications.

  If the environment doesn't support subagents, run the four lenses as separate sequential passes — one dimension at a time, never blended into one read-through.

**Step 2 — verify high-severity findings.** Every finding rated blocker/major goes to a verifier (subagent where available) prompted to REFUTE it: reproduce the failure, check the claim against the actual code. Refuted findings are dropped; confirmed ones are reported with their evidence. Minor findings pass through labeled `unverified`.

**Step 3 — synthesize.** Dedupe across dimensions, rank by severity, attach the concrete failure scenario and suggested fix to each finding.

## Method rules

1. **Understand intent** first: the linked ticket, PR description, or plan doc. A change can be clean code and still the wrong change.
2. **Verify evidence, don't trust claims**: run the tests yourself; if the PR says "all tests pass", check. Missing evidence (test output, commands run) is itself a finding.
3. **Verdict**: end with mergeable / mergeable-after-fixes / not-mergeable. If nothing is wrong, say so in one line — no filler findings.

## Rules

- The human author stays accountable for merging — you provide findings and a verdict, not the merge decision.
- Review the change, not the author; findings reference code, never people.
- Uncertain findings are questions, not accusations: verify or ask.
