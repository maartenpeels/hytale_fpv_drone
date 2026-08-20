---
name: triage
description: Refine a ticket at the business level until it is unambiguous and executable. Usage - /triage <ticket-id>. Fetches the ticket via the team's tracker CLI/MCP (from CLAUDE.md), verifies it against actual system behavior, asks the user about every unknown instead of guessing, then enriches the ticket and creates subtasks — all strictly business-level: no file links, no technical implementation. Technical mapping is workon's job, via its plan. Use before starting work on any vague or non-trivial ticket.
---

# triage — per-ticket business refinement

> Status: v0 draft.

## Scope rule (the contract with workon)

Triage owns the **what and why**; workon owns the **how**. Everything triage writes to the tracker is business-level: behavior, acceptance criteria, scope boundaries, edge cases — expressed in the domain's language. **Never** file paths, module names, implementation approaches, or technical hints in tickets or subtasks. The technical mapping happens later, in workon's plan document.

## Flow

1. **Fetch**: read the tracker + CLI commands from this project's `CLAUDE.md` (e.g. `jira issue view <id>`, `gh issue view <id>`). Pull the ticket's description, comments, links, labels.
2. **Verify against reality**: you MAY read the codebase and running behavior — but only to check the ticket's claims and sharpen your questions ("the ticket says X happens, but the system actually does Y — which is intended?"). Nothing you learn here goes into the ticket as technical detail.
3. **Ask about every unknown.** Never guess on: ambiguous acceptance criteria, unstated edge cases, conflicting requirements, missing context ("which environment?", "is behavior X intended?"), scope boundaries. Batch questions; present options with a recommendation where you have one.
4. **Enrich**: write the refined description back to the ticket — problem statement, acceptance criteria, known business constraints, out-of-scope list. All business-level.
5. **Decompose** (when the ticket warrants it): create subtasks in the tracker, each a coherent, independently valuable slice of business behavior with its own acceptance criteria — sized so `/workon <subtask>` can complete one in a single session.

## Rules

- Tracker-agnostic: everything tool-specific comes from the project's context file, never hardcoded here.
- Write back to the tracker (that's where the team looks), not to a local file.
- Business language only in everything written to the tracker — a product owner should be able to read every subtask.
- If the ticket is already unambiguous and well-scoped, say so and stop — don't manufacture subtasks.
- If system behavior contradicts the ticket, that's a question for the user, not a silent correction.
