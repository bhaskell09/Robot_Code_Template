# GitHub Copilot Instructions

Rules for AI coding agents working in this repository.

This file is **rules for how to work here**. The description of how the codebase is organized lives
in [ARCHITECTURE.md](../ARCHITECTURE.md) — read it before making structural changes.

This mirrors [CLAUDE.md](../CLAUDE.md). **Keep the two in sync**: a rule added to one belongs in the
other.

The skills in `.github/skills/` are likewise mirrored to `.claude/skills/`, because Claude Code reads
only the latter. **`.github/skills/` is canonical** — edit there and run
`pwsh scripts/sync-skills.ps1`, or the next sync overwrites your change.

---

## Project overview

FRC Team 2067 robot code template. WPILib 2026.2.1 command-based Java, CTRE Phoenix 6 swerve
drivetrain, PathPlanner autonomous, dual Limelight vision. Deploys to roboRIO via GradleRIO.

This is a **template**, not a season repo. No game-specific mechanisms — the reusable core plus one
worked example of each pattern. See [SETUP.md](../SETUP.md) to start a season.

---

## Documentation map

| Read this | When |
|---|---|
| **copilot-instructions.md** (this file) | Always. Rules for working here. |
| **[ARCHITECTURE.md](../ARCHITECTURE.md)** | Before adding a subsystem, command, or anything structural. |
| **[SETUP.md](../SETUP.md)** | Starting a new season, or upgrading WPILib. |
| **[README.md](../README.md)** | Onboarding. Install, clone, git workflow. |
| **[skills/](skills/)** | Step-by-step workflows for the five most common tasks. Canonical copy — mirrored to `.claude/skills/`. |

---

## Context7 documentation lookups

Vendor libraries here (Phoenix 6, WPILib, Limelight, PathPlanner) move faster than training data —
use context7 over guessing at an API from memory. VS Code doesn't get this via a checked-in
`mcp.json`: a local npx-run context7 server emits JSON Schema Draft-07 tool definitions that VS
Code's stricter MCP client rejects outright ("tools have invalid JSON schemas and will be omitted").
Install the recommended **Context7 MCP Server** extension (`.vscode/extensions.json` — VS Code
prompts for it on opening this workspace) instead; it talks to Context7's hosted endpoint and
registers itself, no manual server config needed. Resolving by name is ambiguous (e.g. "WPILib"
alone returns four different matches), so these IDs are pre-resolved — call the docs-query tool with
them directly, skip the resolve step:

| Need | context7 ID |
|---|---|
| Phoenix 6 Java API (method signatures) | `/websites/api_ctr-electronics_phoenix6_stable_java` |
| Phoenix 6 guide (tuning, swerve setup, CAN bus) | `/websites/v6_ctr-electronics_en_stable` |
| Phoenix 6 examples | `/crosstheroadelec/phoenix6-examples` |
| WPILib guide (command-based concepts, units, sim) | `/websites/wpilib_en_stable` |
| WPILib javadoc (exact class/method signatures) | `/websites/github_wpilib` |
| Limelight (`LimelightHelpers.java` is this library) | `/limelightvision/limelightlib-wpijava` |
| PathPlanner (NamedCommands, AutoBuilder, markers) | `/mjansen4857/pathplanner` |

Thin results on one of these — try `/crosstheroadelec/phoenix6-documentation`,
`/wpilibsuite/wpilib-docs`, `/wpilibsuite/allwpilib` (raw source instead of javadoc), or
`/websites/pathplanner_dev` (same content, different source).

---

## IMPORTANT: Bash rules

- **Ask before running any build or deploy command** — `./gradlew build`, `./gradlew deploy`,
  `./gradlew simulateJava`.
- **Never deploy without asking.** Someone may have hands on the robot.
- **Read files before editing them**, especially `Constants.java` and `RobotMap.java`.

## Build & deploy

```
./gradlew build
```

Deploy via WPILib VS Code: **Deploy Robot Code** (Shift+F5). Robot must be connected.

---

## Hard rules

Each of these is silent — nothing fails at compile time, and most do not fail at runtime either.

**Never command a position subsystem to `0` in `end()`.** Leave `end()` empty so Motion Magic holds
the last setpoint. Commanding zero drives a gravity-loaded mechanism to the floor at full authority;
`stopMotor()` drops it. Reference: `SetExamplePositionSubsystem.end()`.

**Motor inversion belongs in the subsystem constructor**, via `setInverted(true/false)`. Never negate
a value inside a command to fix direction.

**Register NamedCommands before `configurePathPlanner()`.** PathPlanner resolves names when the autos
load; anything registered afterward is silently ignored, with no error and no warning. Names are
case-sensitive.

**All dashboard output goes through `TelemetryManager`**, never `SmartDashboard` directly. The
documented exception is the autonomous chooser.

**Public APIs use inches and RPM.** Rotations and RPS stay inside the base classes. Put the unit in
the method name when it is not obvious: `getSpeedRPM()`, `isAtInches()`.

**Do not shadow `isAtSetpoint(double, double)`** in a position subclass with an inches version — name
it `isAtInches`. Same signature with different units gets silently overridden.

**Never `markZeroed()` on a zeroing timeout.** A mechanism wrongly marked zeroed is worse than one
that admits it does not know where it is.

**Mark Epilogue-breaking fields `@NotLogged`.** Vendor types, NT handles, and cached timestamps cause
*compile* errors.

**`Constants.ENABLE_SIGNAL_LOGGER` must be `false` before any competition deploy.** The logger writes
to roboRIO flash and can stall the main loop mid-match.

**`generated/TunerConstants.java` is Tuner X output — never hand-edit.** Drivetrain customizations go
in `CommandSwerveDrivetrain`.

---

## Conventions

- Fields `m_` prefixed, constants `k` prefixed.
- Commands verb-first: `SetX`, `RunX`, `ZeroX`.
- CAN IDs in `RobotMap` only. Tuning values in `Constants` only. Live dashboard values in
  `TunableConstants` only.
- Read tunables in `execute()`, not in a constructor.
- Type commands against the base class (`VelocityControlSystem`) unless you need a subsystem-specific
  method.
- Once a tunable is dialed in, move it to `Constants` and delete the entry.

---

## Adding a motor subsystem

The short version — the `add-motor-subsystem` skill has the full workflow.

1. CAN ID into `RobotMap.canIDs`.
2. A `VelocityControlConstants` or `PositionControlConstants` into `Constants`. Both use a builder:
   `forMotor(canId)`, chain the `with*` calls, `build()`. Order does not matter; omitting a required
   group throws at boot.
3. A class extending `VelocityControlSystem` or `PositionControlSystem`. Three lines: `super(...)`,
   `setDefaultConstants(...)`, `setInverted(...)`.
4. Instantiate in `RobotContainer` and bind it.

Do not duplicate logic that belongs in the base class. If you are copying between subsystems, it
belongs one level up.

---

## Session-end protocol

**When a session surfaces something worth remembering, write it down before finishing.**

| Finding | Goes in |
|---|---|
| A new rule, gotcha, or "never do X" | this file's hard rules — and mirror into `CLAUDE.md` |
| How something is structured, or a reusable pattern | `ARCHITECTURE.md` |
| A repeatable multi-step task | a skill under `.github/skills/`, then `pwsh scripts/sync-skills.ps1` |
| A season-start or upgrade step | `SETUP.md` |

Also check: if you changed a file a skill references, update the skill, and run
`pwsh scripts/sync-skills.ps1` so `.claude/skills/` matches. If you corrected a claim in one doc,
check whether the same claim appears in another.

Prefer explaining *why* over adding another all-caps warning.
