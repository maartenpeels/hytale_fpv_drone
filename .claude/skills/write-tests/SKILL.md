---
name: write-tests
description: Build meaningful test coverage on the paths that matter. Identifies critical untested paths ranked by blast radius, writes tests that capture current behavior, and wires them into the file-scoped feedback loop. Use when doctor flags weak coverage or before delegating work in an under-tested area.
---

# write-tests — safety net builder

> Status: v0 draft.

## Flow

1. **Find what matters**: rank untested code by blast-radius (entry points, money/data-mutating paths, code the team is about to delegate work in) — not by raw coverage numbers.
2. **Capture current behavior**: for legacy code, write characterization tests first (what it does), before opinionated tests (what it should do). If current behavior looks wrong, that's a question for the user, not a silent fix.
3. **Write tests that fail for the right reason**: each test should catch a specific realistic regression. No assertion-free tests, no mocking the thing under test.
4. **Keep them fast**: tests join the file-scoped feedback loop; slow tests get marked/separated so agents can still iterate in seconds.
5. **Report**: what's now covered, what remains risky, suggested next targets.

## Rules

- Match the repo's existing test conventions and framework (from the context file and real examples in the repo).
- Never weaken an existing test to make something pass.
- Flaky tests are findings — report them, don't retry-loop around them.
