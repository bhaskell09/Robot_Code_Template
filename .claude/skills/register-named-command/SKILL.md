---
name: register-named-command
description: Register a new PathPlanner NamedCommand in RobotContainer. Use whenever adding an autonomous action, an event marker command, or anything a .auto file needs to trigger partway along a path — and also when an autonomous routine drives the path correctly but a mechanism never fires, which is almost always an unregistered or misspelled name.
---

# Register Named Command

NamedCommands are how PathPlanner paths trigger robot actions at specific waypoints during autonomous. Registration order is critical — commands registered after `configurePathPlanner()` are silently ignored with no error.

## Step 1: Gather Requirements

Ask the user for:

1. **Command name** — the exact string used in the PathPlanner `.auto` file (case-sensitive, no spaces)
2. **What should it do?** — describe the action (e.g., "spin up the shooter to target RPM, then run the feeder for 2 seconds")
3. **Which subsystems does it use?** — list them so the right instances can be found in RobotContainer

## Step 2: Read Existing Files First

Before writing anything, read:
- `src/main/java/frc/robot/RobotContainer.java` — the registration block, the available subsystem fields, and where `configurePathPlanner()` / `AutoBuilder.buildAutoChooser()` sit
- `src/main/java/frc/robot/commands/ExampleNamedCommand.java` — the reference for how an autonomous action is composed

## Step 3: Build the Command Expression

Use the existing subsystem instances from RobotContainer's fields. Do not instantiate new subsystems
— a second instance of a subsystem means two objects talking to one motor, and the scheduler's
requirement system cannot see the conflict.

**Compose existing commands rather than reimplementing the motion.** Every behavior you need in
autonomous already exists as a teleop command that the drivers have actually tested. A NamedCommand
carrying its own copy of a mechanism's logic is a second copy to keep in sync, and it will not stay
in sync.

Common shapes:

```java
// Simple timed action — a command that never self-terminates needs the timeout
new RunExampleVelocitySubsystem(m_exampleVelocity, 2000.0).withTimeout(3.0)

// Sequence: deploy, run, retract. See ExampleNamedCommand.java.
new SequentialCommandGroup(
    new SetExamplePositionSubsystem(m_examplePosition, MyPositions.DEPLOYED),
    new RunExampleVelocitySubsystem(m_exampleVelocity, 2000.0).withTimeout(2.0),
    new SetExamplePositionSubsystem(m_examplePosition, MyPositions.STOWED)
)

// Spin up in parallel with travel, then act
new SequentialCommandGroup(
    new ParallelCommandGroup(
        new SetExamplePositionSubsystem(m_examplePosition, MyPositions.DEPLOYED),
        new RunExampleVelocitySubsystem(m_exampleVelocity, 2000.0).withTimeout(1.0)
    ),
    new RunExampleVelocitySubsystem(m_exampleVelocity, 2000.0).withTimeout(2.0)
)
```

**Prefer a sensor-gated wait over a fixed timer where a sensor exists**, with a timeout as the
backstop. A fixed delay tuned on a fresh battery is wrong by the fourth match.

For anything longer than a couple of steps, put it in its own class under `commands/` rather than
inline in the registration call — a registration block full of multi-line expressions becomes
unreadable fast.

## Step 4: Insert the Registration

Find the `NamedCommands.registerCommand(...)` block in the `RobotContainer()` constructor. Insert the new line within that block:

```java
NamedCommands.registerCommand("MyCommandName", /* command expression */);
```

## Step 5: Verify Registration Order

Confirm the order in RobotContainer is:
```
1. NamedCommands.registerCommand(...)   ← all registrations
2. NamedCommands.registerCommand(...)   ← including your new one
   ...
3. drivetrain.configurePathPlanner()    ← must come AFTER all registerCommand() calls
4. autoChooser = AutoBuilder.buildAutoChooser()
```

If the new registration ended up after `configurePathPlanner()`, move it above that line.

## Step 6: Match the Name in PathPlanner

The registered string must match the event marker in the `.auto` file **exactly** — same spelling,
same capitalization. Open the auto in the PathPlanner GUI and read the marker name off it rather
than trusting memory.

## Critical Rules

- **Unregistered commands silently no-op.** PathPlanner does not error when an event marker has no matching registration — no compile error, no runtime warning. The path drives perfectly and the mechanism never fires, so it reads as a mechanical failure. If an auto drives right but nothing happens, check registration first.
- **Case-sensitive** — `"ScoreHigh"` is not `"scorehigh"`. The string must match the event marker exactly.
- **Registration must precede `configurePathPlanner()`** — the library reads the registry at that call; anything registered later is not visible to the autonomous loader.
- **Use existing subsystem instances** — all subsystems are already instantiated as fields in RobotContainer. Do not `new` them inside the command expression.
