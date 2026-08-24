---
name: add-position-command
description: Create a command that sends a position-controlled subsystem to a setpoint — an arm, elevator, wrist, hood, deployer, climber, or turret. Use whenever asked to "move the arm to X", "raise the elevator", "set the hood angle", "add a button to deploy the intake", or anything else meaning "put this mechanism at a position and hold it". Also use when an existing position command drops its mechanism on release, hangs without finishing, or needs a preset setpoint added — even if the words "position" or "command" never appear.
---

# Add Position Command

A position command sends a `PositionControlSystem` mechanism to a setpoint and finishes when it
arrives. Motion Magic does the actual motion profiling onboard the TalonFX, so the command is
mostly about *when to command*, *when to stop waiting*, and — most importantly — *what happens when
it ends*.

## Step 1: Gather requirements

Ask the user for:

1. **Command name** — verb-first: `SetArmAngle`, `SetElevatorHeight`, `DeployIntake`.
2. **Which mechanism?**
3. **Setpoint source and units** — a fixed value, a named preset from `Constants`, or a live
   `TunableConstants` value. Confirm the unit: inches for linear, degrees for angular.
4. **Tolerance** — how close counts as arrived. If there is already a `POSITION_TOLERANCE` beside
   the mechanism's other constants, use it.
5. **What should `end()` do — hold, or return to a stow position?** Default is hold. Only take
   "stow" for a mechanism where a known safe position exists and travelling there unattended is
   genuinely safe. Ask explicitly; do not assume.
6. **Timeout.** Always yes. Ask for a duration, defaulting to roughly twice the expected travel time.
7. **Does this mechanism need zeroing?** If it homes against a hard stop, the command must refuse to
   move until it has.

## Step 2: Read these files first

- `src/main/java/frc/robot/commands/PositionBasedCommands/SetExamplePositionSubsystem.java` — the
  reference implementation, with the reasoning in comments
- `src/main/java/frc/robot/commands/PositionBasedCommands/ZeroExamplePositionSubsystem.java` — if
  zeroing is involved
- `src/main/java/frc/robot/subsystems/position/PositionControlSystem.java` — what the base class
  provides, including the zeroing API
- The target subsystem, to confirm its unit wrapper is named `setPosition` / `isAtInches`
- Its constants in `constants/Constants.java`, for travel limits and tolerance

## Step 3: Generate the command

Create it under `src/main/java/frc/robot/commands/PositionBasedCommands/`.

```java
package frc.robot.commands.PositionBasedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.MyMechanismPositions;
import frc.robot.subsystems.position.MyMechanism;

public class SetMyMechanism extends Command {

    private final MyMechanism m_subsystem;
    private final double m_targetInches;
    private final Timer m_timer = new Timer();

    private static final double TIMEOUT_SECONDS = 3.0;

    public SetMyMechanism(MyMechanism subsystem, double targetInches) {
        m_subsystem = subsystem;
        m_targetInches = targetInches;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        // Refuse to move a mechanism that does not know where it is.
        if (!m_subsystem.hasZeroed()) {
            return;
        }

        m_subsystem.setPosition(m_targetInches);
    }

    @Override
    public boolean isFinished() {
        if (!m_subsystem.hasZeroed()) {
            return true;  // nothing was commanded; do not hold the scheduler
        }
        return m_subsystem.isAtInches(m_targetInches, MyMechanismPositions.POSITION_TOLERANCE)
            || m_timer.hasElapsed(TIMEOUT_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        // Deliberately empty — this is how the mechanism HOLDS its position.
        // See the critical rules before changing this.
    }
}
```

Drop the `hasZeroed()` guards for a mechanism that does not home against a hard stop.

**If the setpoint is live** (a tunable, or a tracked target), move the command into `execute()` and
return `false` from `isFinished()` so it keeps servoing:

```java
@Override
public void execute() {
    m_subsystem.setPosition(TunableConstants.getMyMechanismInches());
}

@Override
public boolean isFinished() { return false; }
```

## Step 4: Bind it

In `RobotContainer.configureBindings()`, under the **operator controller** section:

```java
m_operatorController.b().onTrue(
    new SetMyMechanism(m_myMechanism, MyMechanismPositions.DEPLOYED));
m_operatorController.x().onTrue(
    new SetMyMechanism(m_myMechanism, MyMechanismPositions.STOWED));
```

`onTrue`, not `whileTrue` — the command finishes on arrival, and the mechanism holds afterward.
Using `whileTrue` would call `end()` on release, which for a "return to stow" variant means the
mechanism moves the instant the driver lets go.

Give presets names in `Constants` rather than passing bare numbers at the binding. `DEPLOYED` still
means something in March; `11.5` does not.

## Critical rules

**Command the setpoint once, in `initialize()`.** Motion Magic generates and follows the trapezoidal
profile onboard the TalonFX. Re-commanding the same target every loop adds CAN traffic and changes
nothing. Move it to `execute()` only when the target genuinely changes mid-command.

**`end()` stays empty to hold position.** This is the single most important rule here. Motion Magic
keeps servoing to the last commanded setpoint after the command ends, and the motor is in Brake
neutral mode, so *doing nothing* is what keeps an arm up.

Adding `setPosition(0)` drives a gravity-loaded mechanism to the floor at full authority. Adding
`stopMotor()` drops it. Both look like tidy cleanup and both break hardware. If cleanup instinct
says something should go in `end()`, that instinct is wrong for this class of mechanism.

**The "return to stow" variant is a different behavior, not a tidier one.** It is legitimate:

```java
@Override
public void end(boolean interrupted) {
    m_subsystem.setPosition(MyMechanismPositions.STOWED);
}
```

But only when the stow position is genuinely safe to reach from anywhere in the travel, including
when the command was interrupted mid-motion by something going wrong. Use it deliberately, and never
as the default.

**Always add a timeout.** A jammed, mis-tuned, or un-zeroed mechanism never satisfies `isFinished()`.
Without a ceiling the command hangs forever, and in autonomous it hangs the whole routine. Finishing
late beats never finishing.

**Compare in the mechanism's own units, via `isAtInches`.** Do not call the base class's
`isAtSetpoint(double, double)` with an inches value — it compares *rotations*, and the mismatch is
silent. If the subsystem lacks an `isAtInches`, add one rather than overloading `isAtSetpoint`.

**Guard motion on `hasZeroed()`** for any mechanism that homes to a hard stop. Commanding a position
off an untrusted encoder sends the mechanism somewhere nobody predicted, at full authority. If no
zeroing command exists yet, use the **add-zeroing-command** skill first.

## After completion

- Running this in autonomous? Use the **register-named-command** skill.
- Mechanism needs homing and has none? Use the **add-zeroing-command** skill.
