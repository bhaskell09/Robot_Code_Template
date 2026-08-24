---
name: session-end-writeup
description: Route findings from this session into the right doc before finishing — CLAUDE.md, ARCHITECTURE.md, a skill, or SETUP.md. Use at the end of a session that surfaced a new rule, gotcha, reusable pattern, repeatable task, or season-start/upgrade step, or whenever the user says something like "write that down", "remember this for next time", or "update the docs before we wrap up".
---

# Session-End Writeup

This repo's docs drift when findings stay in chat logs instead of getting written down. This skill
runs the "Session-end protocol" from `CLAUDE.md` so it actually happens instead of being skipped.

## Step 1: Identify what's worth keeping

Look back over the session for things that would help the *next* session, not things specific to
this one. Ask, for each candidate: would a future Claude (or a future teammate) get this wrong
without being told?

Worth keeping:
- A new rule, gotcha, or "never do X" — especially anything silent (no compile error, no runtime
  warning) that cost time to track down
- A structural pattern worth reusing elsewhere in the codebase
- A multi-step task that was done manually but will come up again
- A season-start or WPILib-upgrade step that isn't already in SETUP.md

Not worth keeping: task-specific details, one-off debugging steps, anything already documented.

If nothing from the session clears that bar, say so and stop — don't manufacture a doc update to
have something to show.

## Step 2: Route each finding

Use this table (from `CLAUDE.md`) to decide where each finding goes:

| Finding | Goes in |
|---|---|
| A new rule, gotcha, or "never do X" | **CLAUDE.md** hard rules — and mirror it into `.github/copilot-instructions.md` |
| How a part of the codebase is structured, or a pattern worth reusing | **ARCHITECTURE.md** |
| A repeatable multi-step task | a skill under `.github/skills/`, then `pwsh scripts/sync-skills.ps1` |
| A season-start or upgrade step | **SETUP.md** |

A single session can produce findings for more than one of these — route each independently.

## Step 3: Write the update

- **Hard rules in CLAUDE.md**: follow the existing format — the rule stated plainly in bold, then
  the reasoning in prose underneath. Prefer explaining *why* over adding another all-caps warning;
  a rule with its reasoning attached survives being read by someone who thinks they know better.
  Mirror the same rule into `.github/copilot-instructions.md` — worded to fit that file's voice, not
  copy-pasted, since the two are deliberately worded differently.
- **ARCHITECTURE.md**: add it under the relevant existing section, or a new section if none fits.
  Keep it game-agnostic — season-specific detail belongs in SETUP.md or the season's own code, not
  here.
- **A skill**: edit the canonical copy in `.github/skills/<name>/SKILL.md` — never `.claude/skills/`
  directly, since that copy is overwritten by the sync script.
- **SETUP.md**: add the step in the sequence it actually needs to happen.

## Step 4: Three checks before finishing

- **If a skill references a file you changed this session, update the skill.** A skill pointing at
  a file that no longer exists is worse than no skill.
- **If you edited any skill, run the sync script**:
  ```
  pwsh scripts/sync-skills.ps1
  ```
- **If you corrected something in one doc, check whether the same claim appears in another.**
  CLAUDE.md, ARCHITECTURE.md, and copilot-instructions.md overlap by design; run a quick search for
  the old claim across all three so the correction doesn't leave a contradiction behind.

## Step 5: Confirm before finishing

Summarize what changed and where, so the user can review it before it's treated as settled — this
is documentation other sessions will trust, so a wrong or overstated rule written here is worse than
not writing one at all.
