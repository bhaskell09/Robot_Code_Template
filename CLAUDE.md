# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in this repository.

This file is **rules for how to work here**. It is not a description of the codebase — that lives in
[ARCHITECTURE.md](ARCHITECTURE.md), which you should read before making structural changes.

---

# FRC Team 2067 — Robot Code Template

WPILib 2026.2.1 command-based Java robot. CTRE Phoenix 6 swerve drivetrain, PathPlanner autonomous,
dual Limelight vision. Deploys to roboRIO via GradleRIO.

This is a **template**, not a season repo. It contains no game-specific mechanisms — just the
reusable core plus one worked example of each pattern. See [SETUP.md](SETUP.md) to start a season.

---

## Documentation map

| Read this | When |
|---|---|
| **CLAUDE.md** (this file) | Always. Rules for working in this repo. |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Before adding a subsystem, command, or anything structural. How the codebase is organized and why. |
| **[SETUP.md](SETUP.md)** | Starting a new season, or upgrading WPILib. |
| **[README.md](README.md)** | Onboarding a new programmer. Install, clone, git workflow. |
| **[.claude/skills/](.claude/skills/)** | Step-by-step workflows for the five most common tasks. Mirrored from `.github/skills/` — see below. |
| **[Systems_Check.md](Systems_Check.md)** | Pre-match pit checklist. |

### Two mirrors to keep in sync

Copilot and Claude Code each read their own directory and will not look in the other's, so two pairs
of files are deliberately duplicated:

| Canonical | Mirror | How |
|---|---|---|
| `.github/copilot-instructions.md` | `CLAUDE.md` (this file) | By hand — the two are worded differently on purpose |
| `.github/skills/` | `.claude/skills/` | `pwsh scripts/sync-skills.ps1` |

**Edit skills in `.github/skills/`, then run the sync script.** Editing the `.claude/` copy directly
means the next sync silently overwrites your change.

```bash
pwsh scripts/sync-skills.ps1          # copy .github/skills -> .claude/skills
pwsh scripts/sync-skills.ps1 -Check   # report drift, exit non-zero if any
```

---

## IMPORTANT: Bash rules

- **Ask before running any build or deploy command.** This includes `./gradlew build`,
  `./gradlew deploy`, and `./gradlew simulateJava`.
- **Never deploy without asking.** A deploy while someone is working on the robot is a safety issue,
  not just an inconvenience.
- **Read files before editing them.** Especially `Constants.java` and `RobotMap.java`, where a value
  you did not expect is usually there for a reason.

## Build & deploy

```
./gradlew build
```

Deploy via WPILib VS Code: **Deploy Robot Code** (Shift+F5). Robot must be connected.

---

## Hard rules

These are the mistakes that have actually cost us matches or hardware. Each one is silent — nothing
fails at compile time, and most do not fail at runtime either.

**Never command a position subsystem to `0` in `end()`.** Leave `end()` empty so Motion Magic holds
the last setpoint. Commanding zero drives a gravity-loaded mechanism to the floor at full authority;
`stopMotor()` drops it. See `SetExamplePositionSubsystem.end()` for the reference and the reasoning.

**Motor inversion belongs in the subsystem constructor**, via `setInverted(true/false)`. Never negate
a value inside a command to correct direction — the next command driving that motor has to remember
to do the same, and one of them will not.

**Register NamedCommands before `configurePathPlanner()`.** PathPlanner resolves names when the
autos load. Anything registered afterward is silently ignored: no compile error, no runtime warning,
the event marker just does nothing and the auto looks like a mechanical failure. Names are
case-sensitive and must match the `.auto` file exactly.

**All dashboard output goes through `TelemetryManager`**, never `SmartDashboard` directly. The one
documented exception is the autonomous chooser, which must publish even when telemetry is disabled.

**Public APIs use inches and RPM.** Rotations and RPS stay inside the base classes. When a method's
unit is not obvious from its signature, put it in the name.

**Do not shadow `isAtSetpoint(double, double)`** in a position subclass with an inches-domain
version. Name it `isAtInches`. Same signature, different units, silently overridden — the caller
reads rotations and gets inches.

**Never `markZeroed()` on a zeroing timeout.** A mechanism wrongly marked as zeroed is worse than
one that admits it does not know where it is.

**Mark Epilogue-breaking fields `@NotLogged`.** Vendor types, NT handles, and cached timestamps
cause *compile* errors, not runtime ones.

**`Constants.ENABLE_SIGNAL_LOGGER` must be `false` before any competition deploy.** The logger
writes to roboRIO flash and can stall the main loop mid-match.

**`generated/TunerConstants.java` is Tuner X output — never hand-edit it.** Changes are lost on the
next regeneration. Drivetrain customizations go in `CommandSwerveDrivetrain`.

---

## Conventions

- Fields are `m_` prefixed; constants are `k` prefixed.
- Commands are verb-first: `SetX`, `RunX`, `ZeroX`.
- CAN IDs go in `RobotMap` only. Tuning values go in `Constants` only. Live dashboard values go in
  `TunableConstants` only.
- Read tunables in `execute()`, not in a constructor — a constructor runs once at boot.
- Type commands against the base class (`VelocityControlSystem`) rather than the concrete subsystem
  unless you need a method only the subsystem has.
- Once a tunable is dialed in, move it to `Constants` and delete the entry.

---

## Session-end protocol

**When a session surfaces something worth remembering, write it down before finishing.**

This is what keeps these files useful. The 2026 repo drifted because findings stayed in people's
heads and in chat logs, and three documents slowly started contradicting each other.

Where it goes:

| Finding | Goes in |
|---|---|
| A new rule, gotcha, or "never do X" | **CLAUDE.md** hard rules — and mirror it into `.github/copilot-instructions.md` |
| How a part of the codebase is structured, or a pattern worth reusing | **ARCHITECTURE.md** |
| A repeatable multi-step task | a skill under `.github/skills/`, then `pwsh scripts/sync-skills.ps1` |
| A season-start or upgrade step | **SETUP.md** |

Three things to check while you are there:

- **If you changed a file a skill references, update the skill.** A skill pointing at a file that no
  longer exists is worse than no skill.
- **If you edited any skill, run `pwsh scripts/sync-skills.ps1`** so both copies match.
- **If you corrected something in one doc, check whether the same claim appears in another.** These
  files overlap by design; they drift by accident.

Prefer explaining *why* over adding another all-caps warning. A rule with its reasoning attached
survives being read by someone who thinks they know better — which, eventually, is everyone.
