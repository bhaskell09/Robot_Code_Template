---
name: add-velocity-command
description: Create a command that drives a velocity-controlled subsystem — a flywheel, shooter wheel, roller, intake, feeder, indexer, kicker, or conveyor. Use whenever asked to "run the X motor", "spin up X", "make a command for my shooter wheels", "add a button to run the intake", or anything else that means turning a motor at a speed. Also use when an existing velocity command needs a target changed, a timeout added, or live dashboard tuning wired in — even if the words "velocity" or "command" never appear.
---

# Add Velocity Command

A velocity command drives one or more `VelocityControlSystem` subsystems at a target RPM. The
subsystem already handles PID, current limits, and telemetry — the command's only jobs are deciding
*what speed*, *when to start and stop*, and *what happens on the way out*.

## Step 1: Gather requirements

Ask the user for:

1. **Command name** — verb-first, e.g. `RunIntake`, `SpinUpShooter`. `Run*` for continuous,
   `SpinUp*` for something that finishes once at speed.
2. **Which subsystem(s)?** One command can drive several motors together — a shooter pair, a
   two-stage indexer.
3. **Where does the target RPM come from?**
   - a fixed value passed to the constructor,
   - `TunableConstants`, re-read every loop so it can be tuned from the dashboard,
   - or computed each loop from something else (distance, a sensor).
4. **When should it finish?**
   - never, running while a button is held (`whileTrue`),
   - once the motor reaches speed (a spin-up step inside a sequence),
   - or after a fixed duration (usually better expressed as `.withTimeout()` at the binding).
5. **What should `end()` do?** Almost always `stopMotor()`. Ask only if the mechanism should keep
   running after the command releases.
6. **Do any motors run opposite each other?** If so, that is an inversion question, not a command
   question — see the critical rules.

## Step 2: Read these files first

- `src/main/java/frc/robot/commands/VelocityBasedCommands/RunExampleVelocitySubsystem.java` — the
  reference implementation, with the reasoning in comments
- `src/main/java/frc/robot/subsystems/velocity/VelocityControlSystem.java` — what the base class
  already gives you
- `src/main/java/frc/robot/constants/TunableConstants.java` — if a tunable is involved
- The target subsystem itself, to see whether it adds anything beyond the base class

## Step 3: Generate the command

Create it under `src/main/java/frc/robot/commands/VelocityBasedCommands/`.

```java
package frc.robot.commands.VelocityBasedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunableConstants;
import frc.robot.subsystems.velocity.VelocityControlSystem;

public class RunMyMechanism extends Command {

    private final VelocityControlSystem m_subsystem;
    private final double m_targetRPM;
    private final boolean m_useTunableConstants;

    public RunMyMechanism(VelocityControlSystem subsystem, double targetRPM) {
        this(subsystem, targetRPM, false);
    }

    public RunMyMechanism(VelocityControlSystem subsystem, double targetRPM,
                          boolean useTunableConstants) {
        m_subsystem = subsystem;
        m_targetRPM = targetRPM;
        m_useTunableConstants = useTunableConstants;
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        double target = m_useTunableConstants
            ? TunableConstants.getMyMechanismRPM()
            : m_targetRPM;
        m_subsystem.setSpeed(target);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
```

For a spin-up step that belongs inside a sequence, replace `isFinished()`:

```java
private static final double TOLERANCE_RPM = 50.0;

@Override
public boolean isFinished() {
    return m_subsystem.isAtSetpoint(m_targetRPM, TOLERANCE_RPM);
}
```

Add a timeout when this runs in autonomous — a motor that never reaches speed because something is
jammed will otherwise hang the routine.

## Step 4: Bind it

In `RobotContainer.configureBindings()`, under the **operator controller** section — mechanism
controls belong there so the driver's controller stays dedicated to driving.

```java
m_operatorController.a().whileTrue(new RunMyMechanism(m_myMechanism, 2000.0, true));
```

`whileTrue` runs while held and calls `end()` on release. Use `onTrue` only for something that
finishes on its own.

## Critical rules

**Type the field as `VelocityControlSystem`, not the concrete subsystem.** Any velocity mechanism
then satisfies it, so one command drives a flywheel, a roller, or a feeder without modification —
and it survives into next season, when the mechanisms change but the base class does not. Use the
concrete type only when you need a method that subsystem alone defines.

**Call `addRequirements()` for every subsystem the command touches.** Without it, two commands can
drive the same motor simultaneously and it obeys whichever wrote last. That bug looks electrical and
costs an afternoon in the pit.

**`end()` calls `stopMotor()`.** Safe here because a flywheel coasts down harmlessly — which is
exactly the opposite of a position mechanism, where cutting output drops a loaded arm. Do not carry
habits between the two.

**Read tunables inside `execute()`, never in the constructor.** The constructor runs once at robot
boot, so a value captured there is frozen until the next deploy — which defeats the point of having
it on the dashboard.

**Never negate a speed inside the command to fix a motor's direction.** Inversion is a property of
how the motor is mounted, so it belongs in the subsystem constructor via `setInverted()`. Fix it in
a command and every other command driving that motor has to remember the same negation, and one of
them eventually will not. If two motors must spin opposite ways as a pair, use
`followMotor(leader, opposed = true)`.

**Make the RPM source explicit.** A command that accepts a target and then silently overwrites it
from the dashboard makes every call site read like a lie. The `useTunableConstants` flag exists so
the reader can tell which value wins.

## After completion

- Running this in autonomous? Use the **register-named-command** skill.
- Once the tuned RPM is settled, move it from `TunableConstants` into `Constants` and delete the
  entry — a tunable that outlives its tuning session is a value nobody can find in the code.
