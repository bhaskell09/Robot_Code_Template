# Starting a Season

What to do with this template in the first week of January, in order.

If you are looking for how the code is organized, that is [ARCHITECTURE.md](ARCHITECTURE.md). If you
are a new programmer setting up a laptop, start with [README.md](README.md).

---

## 1. Create the season repo

```bash
git clone https://github.com/applepi-2067/Robot_Code_Template.git 2027_Robot
cd 2027_Robot
rm -rf .git
git init
git add .
git commit -m "Initial commit from Robot_Code_Template"
```

Removing `.git` starts clean history rather than carrying the template's. Then create the new repo on
GitHub and push.

---

## 2. Upgrade to the new WPILib year

Do this before writing any code — a mid-season upgrade is far more painful than a day-one one.

1. **Install the new WPILib VS Code release** from
   [github.com/wpilibsuite/allwpilib/releases](https://github.com/wpilibsuite/allwpilib/releases).
   It installs alongside the old one; use the new shortcut.

2. **`build.gradle`** — bump the GradleRIO plugin:
   ```groovy
   id "edu.wpi.first.GradleRIO" version "2027.X.X"
   ```

3. **`settings.gradle`** — bump the year:
   ```groovy
   String frcYear = '2027'
   ```

4. **`.wpilib/wpilib_preferences.json`** — set `"projectYear": "2027"`. Confirm `"teamNumber"` is
   still 2067.

5. **Vendordeps.** Delete the old JSONs from `vendordeps/` and re-add each from its current URL via
   *WPILib: Manage Vendor Libraries → Install new libraries (online)*:

   | Library | URL |
   |---|---|
   | Phoenix 6 | `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc<YEAR>-latest.json` |
   | PathPlanner | `https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json` |
   | WPILib New Commands | bundled — re-add via *Manage Vendor Libraries* |

6. **`LimelightHelpers.java`** is a vendored single file, not a vendordep. Replace it wholesale from
   [the LimelightLib repo](https://github.com/LimelightVision/limelightlib-wpijava) if Limelight has
   published an update. Do not hand-patch it.

7. **Build and fix what broke.** WPILib deprecates API between years; the compiler will tell you
   what moved.

```bash
./gradlew build
```

---

## 3. Regenerate the drivetrain constants

`src/main/java/frc/robot/generated/TunerConstants.java` in the template is a **placeholder**. Its
gear ratios, wheel radius, CAN IDs, steer offsets, and slip current describe last year's robot.

1. Open **CTRE Phoenix Tuner X** with the robot on and connected.
2. Run the **Swerve Project Generator**. Work through calibration — it will ask you to spin each
   module and drive the robot.
3. Overwrite `TunerConstants.java` with what it produces.
4. Confirm the CAN IDs it generated match `RobotMap.canIDs.Drivetrain`. Update `RobotMap` to match
   Tuner, not the other way around.

**Never hand-edit `TunerConstants.java`.** Anything you change there is lost the next time it is
regenerated. Drivetrain customizations belong in `CommandSwerveDrivetrain.java`.

---

## 4. Fill in your hardware

**`constants/RobotMap.java`** — replace the `ExampleVelocitySubsystem` and
`ExamplePositionSubsystem` entries with your mechanisms. Keep the numbering blocks (intake 20-29,
handling 30-39, scoring 40-49, superstructure 50-59) and leave gaps.

**`constants/Constants.java`** — replace the two example constants objects. Both constructors are
positional, so count the arguments against the constructor signature.

**`constants/TunableConstants.java`** — replace the example entries with values you actually expect
to tune from the dashboard.

**Limelights** — if the camera hostnames changed, update `VisionConstants.LIMELIGHT_FRONT_NAME` and
`LIMELIGHT_BACK_NAME` to match what is set in each Limelight's web UI. Re-evaluate
`USE_MEGA_TAG_2` on this year's hardware and firmware rather than inheriting last year's verdict.

---

## 5. Add your first subsystem

Use the **add-motor-subsystem** skill, or follow the checklist at the bottom of
[ARCHITECTURE.md](ARCHITECTURE.md).

The short version:

1. CAN ID into `RobotMap`
2. Constants object into `Constants`
3. A class extending `VelocityControlSystem` or `PositionControlSystem` — about three lines
4. Instantiate in `RobotContainer`, bind to a button on the operator controller
5. Write a command with **add-velocity-command** or **add-position-command**

Before deleting the examples, read them. The comments explain choices that look arbitrary until
something breaks — particularly why `SetExamplePositionSubsystem.end()` is empty.

---

## 6. Delete the examples

Once you have real mechanisms:

```
src/main/java/frc/robot/subsystems/velocity/ExampleVelocitySubsystem.java
src/main/java/frc/robot/subsystems/position/ExamplePositionSubsystem.java
src/main/java/frc/robot/commands/VelocityBasedCommands/RunExampleVelocitySubsystem.java
src/main/java/frc/robot/commands/PositionBasedCommands/SetExamplePositionSubsystem.java
src/main/java/frc/robot/commands/PositionBasedCommands/ZeroExamplePositionSubsystem.java
src/main/java/frc/robot/commands/ExampleNamedCommand.java
```

Then remove their entries from `Constants`, `RobotMap`, and `RobotContainer`, and update the
reference paths in the five skills under `.github/skills/` to point at real files. A skill pointing
at a file that no longer exists is worse than no skill.

Then mirror the edits so Claude Code sees them too:

```bash
pwsh scripts/sync-skills.ps1
```

---

## 7. Season housekeeping

**Update `Systems_Check.md`** with this year's mechanisms and button bindings. Do it before the first
competition, not at it.

**Set up PathPlanner.** Autos and paths go in `src/main/deploy/pathplanner/`. The template ships an
empty `autos/` and `paths/` with the settings file — open the PathPlanner GUI and point it at the
project.

**Keep the docs current.** When a session surfaces a new gotcha, write it down. See the session-end
protocol in [CLAUDE.md](CLAUDE.md). The 2026 repo drifted because findings stayed in people's heads
until three documents were quietly contradicting each other.

---

## Before every competition

- [ ] `Constants.ENABLE_SIGNAL_LOGGER = false` — the logger writes to roboRIO flash and can stall the
      main loop mid-match
- [ ] `TelemetryManager.setEnabled(false)` in the `RobotContainer` constructor
- [ ] Every mechanism's zeroing verified on the practice field, not just at the shop
- [ ] Every NamedCommand in every auto actually registered — an unregistered name fails silently
- [ ] `Systems_Check.md` matches the robot as built

---

## Upgrade reference

Everything that carries a year, in one list:

| File | What to change |
|---|---|
| `build.gradle` | GradleRIO plugin version |
| `settings.gradle` | `frcYear` |
| `.wpilib/wpilib_preferences.json` | `projectYear` (and confirm `teamNumber`) |
| `vendordeps/*.json` | Delete and re-add all three |
| `LimelightHelpers.java` | Replace wholesale if updated |
| `generated/TunerConstants.java` | Regenerate from Tuner X |
| `CLAUDE.md`, `.github/copilot-instructions.md` | The WPILib version in the project overview |
