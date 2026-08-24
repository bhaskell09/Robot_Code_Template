# Robot Code Template — FRC Team 2067

The starting point for a new season's robot code.

WPILib 2026.2.1 command-based Java, CTRE Phoenix 6 swerve drivetrain, PathPlanner autonomous, dual
Limelight vision. Deploys to roboRIO via GradleRIO.

**This is a template, not a robot.** It contains no game-specific mechanisms — just the reusable core
that carries over every year, plus one worked example of each pattern so you can see how a mechanism
is supposed to be built before you build one.

Clone it, run `./gradlew build`, and you have a swerve robot that drives, sees AprilTags, and runs
PathPlanner autos.

---

## Where to go next

| You are | Read |
|---|---|
| Starting the new season | **[SETUP.md](SETUP.md)** |
| Adding a subsystem or command | **[ARCHITECTURE.md](ARCHITECTURE.md)** |
| New to the team | This file, then ARCHITECTURE.md |
| An AI coding agent | **[CLAUDE.md](CLAUDE.md)** |
| Prepping for a match | **[Systems_Check.md](Systems_Check.md)** |

---

## Useful links

**WPILib**
- [WPILib docs](https://docs.wpilib.org/en/stable/) — the manual for everything
- [Command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) — read this before writing a command
- [Releases](https://github.com/wpilibsuite/allwpilib/releases)

**CTRE Phoenix 6**
- [Phoenix 6 docs](https://v6.docs.ctr-electronics.com/en/stable/)
- [Swerve setup / Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html)
- [API reference](https://api.ctr-electronics.com/phoenix6/release/java/)

**Vision**
- [Limelight docs](https://docs.limelightvision.io/)
- [LimelightHelpers](https://github.com/LimelightVision/limelightlib-wpijava) — the vendored file in `src/`

**Path planning**
- [PathPlanner docs](https://pathplanner.dev/home.html)

**Community**
- [Chief Delphi](https://www.chiefdelphi.com/) — where FRC teams ask each other things
- [FIRST Robotics Competition](https://www.firstinspires.org/robotics/frc)

---

## Getting started

### Software installation

1. **WPILib 2026.2.1**
   - Download the [latest WPILib release](https://github.com/wpilibsuite/allwpilib/releases)
   - Windows: get the `.iso`, mount it, run `WPILibInstaller.exe`
   - This installs its own copy of VS Code, configured for FRC. Use that one, not a VS Code you
     already had.

2. **FRC Game Tools**
   - [Download from NI](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)
   - Includes the Driver Station. Enter team number **2067** on the Setup tab.

3. **Phoenix Tuner X**
   - [Download from CTRE](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/index.html)
   - Needed for device firmware, CAN diagnostics, and generating the swerve constants.

4. **Git** — [git-scm.com/downloads](https://git-scm.com/downloads)

5. **A GitHub account** — sign up at [github.com](https://github.com), then ask team leadership to
   add you to the `applepi-2067` organization.

### Cloning

1. Open **WPILib VS Code** from your desktop
2. **Source Control** tab (Ctrl+Shift+G) → **Clone Repository** → **Clone from GitHub**
3. Authorize VS Code, pick the repo, choose a directory (e.g. `Documents\FRC\`)
4. Wait for dependencies to download and the first build to finish — watch the Terminal panel

If the first build fails, it is almost always a vendordep that has not downloaded yet. Run
`./gradlew build` again with an internet connection.

---

## Using VS Code

Most actions go through the
[Command Palette](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/vscode-basics.html#command-palette)
(Ctrl+Shift+P).

| Command | What it does |
|---|---|
| **WPILib: Build Robot Code** | Compiles |
| **WPILib: Deploy Robot Code** (Shift+F5) | Compiles and pushes to the roboRIO — must be connected |
| **WPILib: Simulate Robot Code** | Runs on your laptop; needs a Driver Station to enable |

Simulation is underused. You can catch most logic errors without ever touching the robot, and the
robot is usually busy.

---

## Using Git

Easier through VS Code's Source Control panel, but the commands are:

```bash
# Get new branches and the latest changes
git fetch
git pull

# Branch for your work
git checkout -b my-feature

# Commit
git add .
git commit -m "Description of changes"

# Push
git push origin my-feature
```

**Practices that matter:**

- Fetch and pull before starting work. Merge conflicts are cheap to avoid and expensive to resolve
  at 11pm before a competition.
- Branch for every feature. Never commit directly to `main`.
- Write commit messages that say *why*, not *what* — the diff already shows what.
- Open a pull request and get a review before merging. The
  [PR template](.github/pull_request_template.md) lists what to include.

More: [WPILib Git guide](https://docs.wpilib.org/en/stable/docs/software/basic-programming/git-getting-started.html)

---

## Project structure

```
├── .claude/
│   └── skills/                    Mirror of .github/skills, for Claude Code
├── .github/
│   ├── copilot-instructions.md    Rules for AI coding agents
│   ├── pull_request_template.md
│   └── skills/                    Step-by-step workflows (canonical copy)
├── scripts/
│   └── sync-skills.ps1            Mirrors .github/skills -> .claude/skills
├── src/main/java/frc/robot/
│   ├── Main.java                  Entry point — do not modify
│   ├── Robot.java                 TimedRobot lifecycle
│   ├── RobotContainer.java        Subsystems, button bindings, autonomous wiring
│   ├── Telemetry.java             Swerve telemetry (Tuner X)
│   ├── LimelightHelpers.java      Vendored — replace wholesale, never patch
│   ├── constants/
│   │   ├── Constants.java         Tuning values
│   │   ├── RobotMap.java          CAN IDs, DIO, PWM, bus names
│   │   └── TunableConstants.java  Live dashboard values
│   ├── generated/
│   │   └── TunerConstants.java    Tuner X output — regenerate, never hand-edit
│   ├── subsystems/
│   │   ├── CommandSwerveDrivetrain.java
│   │   ├── Vision.java
│   │   ├── velocity/              VelocityControlSystem + subclasses
│   │   └── position/              PositionControlSystem + subclasses
│   ├── commands/
│   │   ├── VelocityBasedCommands/
│   │   └── PositionBasedCommands/
│   └── utils/
│       └── TelemetryManager.java  All dashboard output goes through here
├── src/main/deploy/pathplanner/   Autos and paths
├── vendordeps/                    Phoenix 6, PathPlanner, WPILib New Commands
├── ARCHITECTURE.md                How the codebase is organized, and why
├── CLAUDE.md                      Rules for AI coding agents
├── SETUP.md                       Starting a season
└── Systems_Check.md               Pre-match pit checklist
```

---

## Development guidelines

**Code style**

- Fields `m_` prefixed, constants `k` prefixed
- Commands verb-first: `SetX`, `RunX`, `ZeroX`
- CAN IDs in `RobotMap` only, tuning values in `Constants` only, live values in `TunableConstants`
  only
- All dashboard output through `TelemetryManager`, never `SmartDashboard` directly

**Safety first**

- **Never command a position subsystem to `0` in `end()`.** Leave it empty so Motion Magic holds the
  setpoint. Commanding zero drives a gravity-loaded mechanism to the floor.
- Never deploy while someone has hands on the robot.
- Test in simulation before testing on hardware.
- A mechanism that has not been zeroed should refuse to move, not guess.

**Adding a motor subsystem**

Three lines, if you do it the way the base classes intend. See the checklist at the bottom of
[ARCHITECTURE.md](ARCHITECTURE.md), or use the `add-motor-subsystem` skill.

---

## Resources for new members

- [WPILib zero-to-robot](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html)
- [Command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [Java basics](https://docs.oracle.com/javase/tutorial/java/index.html) — if Java is new to you
- Read `ExampleVelocitySubsystem.java` and `ExamplePositionSubsystem.java` in this repo. They are
  short and the comments explain the reasoning.

Ask questions. Every person on this team has broken something expensive by not asking one.

---

## Contact

Reach out to team leadership or a programming mentor. If you are stuck on something for more than
about twenty minutes, that is the signal to ask rather than the signal to keep going.
