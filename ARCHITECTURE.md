# Robot Software Architecture

How this codebase is organized and why. This is the reference for **structure** — what the pieces
are, how they fit together, and which patterns to reach for.

For **rules about how to work in this repo** (what to ask before doing, what never to do), see
[CLAUDE.md](CLAUDE.md) and [.github/copilot-instructions.md](.github/copilot-instructions.md). For
**getting a new season started**, see [SETUP.md](SETUP.md).

This document is game-agnostic on purpose. Everything here applies to any CTRE Phoenix 6 + WPILib
command-based robot, regardless of the year. When a season's game logic gets added, it goes in the
places described under [Where season-specific code goes](#where-season-specific-code-goes).

---

## Table of contents

- [Design principles](#design-principles)
- [Two motor base classes](#two-motor-base-classes)
- [StatusSignal caching](#statussignal-caching)
- [Telemetry rate limiting](#telemetry-rate-limiting)
- [Command patterns](#command-patterns)
- [Load monitoring patterns](#load-monitoring-patterns)
- [Constants organization](#constants-organization)
- [TelemetryManager](#telemetrymanager)
- [CAN bus topology](#can-bus-topology)
- [Units convention](#units-convention)
- [PathPlanner integration](#pathplanner-integration)
- [Match mode initialization](#match-mode-initialization)
- [Haptic feedback](#haptic-feedback)
- [Epilogue logging](#epilogue-logging)
- [Vision / pose estimation](#vision--pose-estimation)
- [Code style conventions](#code-style-conventions)
- [AdvantageScope and simulation](#advantagescope-and-simulation)
- [Where season-specific code goes](#where-season-specific-code-goes)
- [Template file map](#template-file-map)

---

## Design principles

**Put shared behavior in a base class, not in a copy.** Two abstract subsystems cover nearly every
motor on the robot. A new mechanism should be three lines, not three hundred. If you are copying
logic between subsystems, it belongs one level up.

**One place per fact.** A CAN ID lives in `RobotMap` and nowhere else. A PID gain lives in
`Constants` and nowhere else. Duplicated facts drift, and the copy that drifts is never the one you
are looking at when the robot misbehaves.

**Convert units at the boundary.** Public APIs speak in what humans use — inches, degrees, RPM.
Rotations and RPS stay inside the base classes.

**Prefer failing loudly and stationary over quietly and wrong.** A mechanism that reports "I don't
know where I am" and refuses to move is debuggable. One that guesses is not.

**Make the CAN bus cheap.** Cache signals, batch refreshes, rate-limit telemetry. Bus utilization is
a shared budget, and the symptoms of overrunning it look like everything else breaking.

---

## Two motor base classes

Almost every motor on the robot is either "spin at a speed" or "go to a position." Two abstract
classes cover both, and concrete subsystems just supply a CAN ID, a name, and a constants object.

### VelocityControlSystem — `subsystems/velocity/VelocityControlSystem.java`

Closed-loop velocity via `VelocityVoltage`. **Coast** neutral mode — a flywheel that brakes on
release fights itself and wears the gearbox.

| Member | Notes |
|---|---|
| `setSpeed(double rpm)` | Public API is RPM; converts to RPS internally |
| `getSpeedRPM()` / `getSpeedRPS()` | Both named for their unit, from the cached signal |
| `isAtSetpoint(targetRPM, toleranceRPM)` | Cheap enough to poll every loop |
| `stopMotor()` | |
| `setInverted(boolean)` | `protected` — call from the subclass constructor only |
| `followMotor(leader, opposed)` | Slaves this motor to another; `opposed` for motors facing opposite ways |
| `setDefaultConstants(VelocityControlConstants)` | Applies Slot0 PIDV and current limits |

A concrete subsystem in full:

```java
public class ExampleVelocitySubsystem extends VelocityControlSystem {
    public ExampleVelocitySubsystem() {
        super(Constants.ExampleVelocitySubsystemConstants.kCanId, "ExampleVelocitySubsystem");
        setDefaultConstants(Constants.ExampleVelocitySubsystemConstants);
        setInverted(false);
    }
}
```

### PositionControlSystem — `subsystems/position/PositionControlSystem.java`

Closed-loop position via `MotionMagicVoltage`. **Brake** neutral mode — a position mechanism that
coasts falls.

| Member | Notes |
|---|---|
| `setRotation(double motorRotations)` | Rotation domain; subclasses wrap it in real units |
| `isAtSetpoint(targetRotations, toleranceRotations)` | **Rotations.** Subclasses add their own unit version under a different name |
| `inchesToRotations` / `rotationsToInches` | `protected`; uses `kInchesPerRotation` |
| `setVoltage(double)` | Open-loop escape hatch, used by homing |
| `resetEncoder()` | |
| `getStatorCurrent()` | What stall detection reads |
| `markZeroed()` / `clearZeroed()` / `hasZeroed()` | Zeroing state, see [pattern 4](#pattern-4-hard-stop-zeroing) |
| `setEncoderToHardStop(double mechanismInches)` | Writes the stop's position and marks zeroed |

`periodic()` also watches `hasResetOccurred()`. A TalonFX that browns out and reboots comes back
with its encoder at zero, silently invalidating a homing result; without this check the mechanism
runs against a bogus reference for the rest of the match.

**The unit boundary is the subclass's job.** The base class works in rotations. A concrete
subsystem exposes inches or degrees, and — importantly — names its setpoint check something *other*
than `isAtSetpoint`:

```java
public boolean isAtInches(double targetInches, double toleranceInches) { ... }
```

Declaring an inches version at the base class's `isAtSetpoint(double, double)` signature overrides
it silently. Any caller holding a base-class reference then gets inches semantics while reading code
that says rotations, and nothing about the call site reveals it.

---

## StatusSignal caching

Phoenix 6 returns the *same* `StatusSignal` object every time you call `getVelocity()`. Calling it
repeatedly does not cost a CAN read, but calling `getValueAsDouble()` on a stale signal gives stale
data, and refreshing signals one at a time costs one CAN transaction each.

The pattern: grab the references once in the constructor, set an update frequency per signal, then
batch-refresh them in `periodic()`.

```java
// Constructor — once
m_velocitySignal      = m_motor.getVelocity();
m_voltageSignal       = m_motor.getMotorVoltage();
m_statorCurrentSignal = m_motor.getStatorCurrent();
m_supplyCurrentSignal = m_motor.getSupplyCurrent();

m_velocitySignal.setUpdateFrequency(50);      // used by isAtSetpoint() every loop
m_voltageSignal.setUpdateFrequency(10);       // dashboard only
m_statorCurrentSignal.setUpdateFrequency(10);
m_supplyCurrentSignal.setUpdateFrequency(10);

m_motor.optimizeBusUtilization();  // mutes every signal you did NOT ask for

// periodic() — one CAN transaction instead of four
BaseStatusSignal.refreshAll(m_velocitySignal, m_voltageSignal,
                            m_statorCurrentSignal, m_supplyCurrentSignal);
```

**Choosing a frequency:**

- **50 Hz** — anything read in `isFinished()`, a setpoint comparison, or a detection state machine.
  This is the robot loop rate; slower means acting on stale data.
- **10 Hz** — dashboard-only values. Nothing decides anything based on these.

`optimizeBusUtilization()` is what makes this pay off: it silences every signal the device would
otherwise publish by default, so the bus only carries what you asked for.

A subsystem that needs one signal faster than the base class provides can raise it in its own
constructor:

```java
m_statorCurrentSignal.setUpdateFrequency(50);  // detection runs every loop
```

---

## Telemetry rate limiting

Dashboard values do not need to update at 50 Hz. Nobody can read that fast, and every write costs
NetworkTables bandwidth.

```java
private int m_periodicCounter = 0;
private static final int TELEMETRY_PERIOD = 5;   // every 5 loops = 100 ms

@Override
public void periodic() {
    m_periodicCounter++;
    if (m_periodicCounter % TELEMETRY_PERIOD != 0) return;
    // ... refresh signals and publish
}
```

`TELEMETRY_PERIOD = 5` is a reasonable default, not a law. Raise it for values that change slowly;
lower it while actively debugging something.

**The early return skips the signal refresh too.** That is intentional for telemetry-only signals,
but it means anything that must run every loop — a detection state machine, say — has to run
*before* the return, or refresh its own signal separately.

---

## Command patterns

Four shapes cover nearly everything.

### Pattern 1: one-shot move to position

Reference: `commands/PositionBasedCommands/SetExamplePositionSubsystem.java`

```java
@Override
public void initialize() {
    m_subsystem.setPosition(m_targetInches);   // ONCE
}

@Override
public boolean isFinished() {
    return m_subsystem.isAtInches(m_targetInches, TOLERANCE)
        || m_timer.hasElapsed(TIMEOUT_SECONDS);
}

@Override
public void end(boolean interrupted) {
    // Deliberately empty — this is how the mechanism HOLDS position.
}
```

Three things carry weight here:

**Command the setpoint once, in `initialize()`.** Motion Magic generates and follows the profile
onboard the TalonFX. Re-commanding the same target every loop adds CAN traffic and changes nothing.
Move it to `execute()` only when the target genuinely changes mid-command — a live tunable, or a
tracked target.

**`end()` stays empty.** Motion Magic keeps servoing to the last setpoint after the command ends,
and the motor is in Brake mode, so doing nothing is what keeps an arm up. Adding `setPosition(0)`
drives a gravity-loaded mechanism to the floor at full authority; adding `stopMotor()` drops it.
**This is the most common way to break a mechanism in this codebase.**

A "return to stow" variant is legitimate where a known safe position exists and travelling there
unattended is genuinely safe — but it is a different behavior, not a tidier version of holding, and
it is only safe when the stow position really is reachable from anywhere in the travel.

**Always add a timeout.** A jammed or mis-tuned mechanism never satisfies `isFinished()`, and in
autonomous that hangs the whole routine. Finishing late beats never finishing.

### Pattern 2: continuous velocity loop

Reference: `commands/VelocityBasedCommands/RunExampleVelocitySubsystem.java`

```java
@Override
public void execute() {
    double target = m_useTunableConstants
        ? TunableConstants.getExampleVelocityRPM()
        : m_targetRPM;
    m_subsystem.setSpeed(target);
}

@Override
public void end(boolean interrupted) { m_subsystem.stopMotor(); }

@Override
public boolean isFinished() { return false; }
```

**Type the field as `VelocityControlSystem`, not the concrete subsystem.** One command then drives
any velocity mechanism, and survives into next season when the mechanisms change but the base class
does not. Use the concrete type only when you need a method it alone defines.

**`end()` calls `stopMotor()`** — safe here, because a flywheel coasts down harmlessly. Exactly the
opposite of pattern 1.

**Read tunables in `execute()`, not the constructor.** The constructor runs once at robot boot; a
value captured there is frozen until the next deploy.

**Make the RPM source explicit.** A command that accepts a target and then silently overwrites it
from the dashboard makes every call site read like a lie. The `useTunableConstants` flag says which
one wins.

### Pattern 3: state machine

For multi-phase actions — deploy, wait, run, retract:

```java
private enum Phase { RETRACTING, APPROACHING }
private Phase m_phase;

@Override
public void initialize() {
    m_phase = Phase.RETRACTING;
    m_timer.restart();
}

@Override
public void execute() {
    switch (m_phase) {
        case RETRACTING -> { ...; if (done) m_phase = Phase.APPROACHING; }
        case APPROACHING -> { ... }
    }
}
```

Reset every piece of state in `initialize()`, not at the field declaration. Commands are constructed
once and scheduled many times; state left over from the previous run is a bug that only appears the
second time a button is pressed.

Consider composing existing commands with `SequentialCommandGroup` before writing a state machine —
see `commands/ExampleNamedCommand.java`. Reach for an explicit machine when phases share state or a
transition depends on a sensor.

### Pattern 4: hard-stop zeroing

Reference: `commands/PositionBasedCommands/ZeroExamplePositionSubsystem.java`, and the
`add-zeroing-command` skill.

A relative encoder reads zero wherever the mechanism happened to be sitting at power-on. Homing
fixes that: drive gently into a known physical stop, detect the stall, declare the position known.

**Retract first, then approach.** The mechanism may already be resting against the stop at power-on,
in which case an immediate approach reads stall current instantly — correct by accident, and
indistinguishable from a jam. Backing off first means the stall you detect is one you caused, and it
guarantees a consistent approach direction so backlash lands the same way every time.

**Debounce the stall.** Raw stator current spikes on direction changes and at any sticky spot in the
travel. An undebounced threshold latches onto the first of those and zeroes mid-range. Requiring the
current to stay high for a continuous window (~100 ms) rejects transients while still stopping well
before damage.

**Check the threshold against the stator limit.** If the stall threshold is at or above
`kStatorCurrentLimit`, the limit caps current before the detector ever fires and homing silently
never completes. This shipped once — a 7.5 A limit against a 9.0 A threshold.

**Never `markZeroed()` on timeout.** A mechanism wrongly marked as zeroed is worse than one that
admits it does not know where it is: every setpoint afterward is off by the same unknown amount, and
it surfaces as mysterious mechanical misbehavior rather than an error anyone can act on.

**Guard motion on `hasZeroed()`.** Position commands should refuse to move an un-homed mechanism.

---

## Load monitoring patterns

Stator current is a free load sensor. Two ways to read it.

### Spike detection — discrete events

A game piece passing through a mechanism produces a brief current spike. A two-state machine with a
cooldown counts those events:

```java
private enum SpikeState { IDLE, IN_SPIKE }

// in periodic(), running EVERY loop (raise the signal to 50 Hz first):
double current = m_statorCurrentSignal.refresh().getValueAsDouble();
switch (m_spikeState) {
    case IDLE -> {
        if (current > kSpikeThresholdAmps && Math.abs(getSpeedRPS()) >= kMinDetectionSpeedRps) {
            m_count++;
            m_spikeState = SpikeState.IN_SPIKE;
        }
    }
    case IN_SPIKE -> {
        if (current < kSpikeThresholdAmps) m_spikeState = SpikeState.IDLE;
    }
}
```

The speed gate matters: a stopped or spinning-up motor draws high current for reasons that have
nothing to do with a game piece.

### EMA filter — sustained states

For "is it loaded" rather than "did something pass," an exponential moving average plus hysteresis:

```java
m_filtered += kEmaAlpha * (current - m_filtered);
```

`kEmaAlpha ≈ 0.1` at 50 Hz gives roughly a 200 ms time constant — smooth but still responsive. Use
**separate thresholds for entering and leaving** a state (e.g. enter at 50 A, clear at 15 A) so a
value hovering at the boundary does not chatter, and debounce the transitions on top of that.

---

## Constants organization

Three files, three jobs. Keeping them separate is what makes any given value findable.

### `constants/RobotMap.java` — what is plugged in where

CAN IDs, DIO channels, PWM channels, CAN bus names. Nothing else.

```java
RobotMap.canIDs.ExampleVelocitySubsystem.MOTOR
RobotMap.canBusNames.CANIVORE
```

Numbering leaves gaps by block (drivetrain 0-8, intake 20-29, handling 30-39, scoring 40-49,
superstructure 50-59) so a mid-season mechanism can be added without renumbering.

Never hardcode a bus name as a string literal at a call site — use `RobotMap.canBusNames`.

### `constants/Constants.java` — tuning values

PID gains, current limits, soft limits, tolerances, speeds. Organized as nested static classes, plus
one `VelocityControlConstants` / `PositionControlConstants` instance per motor subsystem.

Those two constructors are **positional**, so argument order is what binds each number to its field
— not the comment beside it. Miscount by one and kP silently becomes kD, with no compiler error and
a mechanism that misbehaves in a way that looks mechanical. Keep the comments aligned and count
twice. (Watch for integer division too: `5/1` is an int expression that truncates. Write `5.0`.)

### `constants/TunableConstants.java` — live dashboard values

Values in the `"Tuning"` NetworkTable, editable while the robot runs.

```java
private static final DoubleEntry exampleVelocityRPM =
    table.getDoubleTopic("ExampleVelocitySubsystem/RPM").getEntry(2000.0);

public static double getExampleVelocityRPM() { return exampleVelocityRPM.get(); }
```

A `DoubleEntry` reads live from NetworkTables on every `.get()`. Nothing pushes or polls on a timer,
and `RobotContainer.periodic()` does not need to do anything for this to work. What matters is
*where* you call the getter — in `execute()` it is live, in a constructor it is frozen at boot.

Specify each default exactly once. Passing one value to `getEntry()` and a different one to
`setDefault()` makes the result depend on whether the topic had been published yet.

**Once a value is tuned, move it to `Constants` and delete the entry.** A tunable that outlives its
tuning session is a value nobody can find in the code and that silently resets if the dashboard
layout is lost.

---

## TelemetryManager

Reference: `utils/TelemetryManager.java`. **All dashboard output goes through this.** Never call
`SmartDashboard` directly.

```java
TelemetryManager.putNumber("Mechanism/RPM", getSpeedRPM());
TelemetryManager.putBoolean("Mechanism/isReady", isAtSetpoint(target, 50));
TelemetryManager.putString("Mechanism/state", m_state.name());
```

Three behaviors earn it its place:

**Change detection.** A value is written only when it differs from the last one published under that
key. Pass `force = true` to bypass.

**Register-once for Sendables.** `putData()` registers on first call only, because `putData`
re-serializes the whole object every time.

**Global kill switch.** `TelemetryManager.setEnabled(false)` short-circuits everything, which is
what you want at competition. Anything that must always publish — the autonomous chooser — goes
directly to `SmartDashboard` and is commented to say why.

The backing map is a plain `HashMap` and is not thread-safe. Every caller today is on the main loop;
if you add a thread, do not publish from it.

---

## CAN bus topology

- **CANivore** (`RobotMap.canBusNames.CANIVORE`) — drivetrain only: drive, steer, CANcoders,
  Pigeon 2.0. Isolating the drivetrain means a mechanism flooding the bus cannot make the robot
  undriveable.
- **roboRIO bus** — everything else.

**CAN IDs are unique per bus, not globally.** Drivetrain IDs 1-8 can safely coexist with RIO-bus
IDs. Keeping them in separate blocks anyway makes a Phoenix Tuner device list readable.

Both buses are monitored in `RobotContainer.updateTelemetry()`. Rising utilization or a climbing
error count is the earliest visible sign of a failing wire or a dying device — usually well before
the mechanism it feeds starts acting up.

---

## Units convention

| Layer | Unit |
|---|---|
| Position subsystem public API | inches (or degrees for angular mechanisms) |
| Position subsystem internals | motor rotations |
| Velocity subsystem public API | RPM |
| Velocity subsystem internals | RPS |
| Pose / field geometry | meters, `wpiBlue` frame |
| Angles in commands | degrees |
| Angles in WPILib geometry | radians (`Rotation2d`) |

Convert at the boundary, in exactly one place. When a method's unit is not obvious from its
signature, put it in the name: `getSpeedRPM()`, `isAtInches()`.

---

## PathPlanner integration

Setup order in the `RobotContainer` constructor:

```java
// 1. Register named commands FIRST
NamedCommands.registerCommand("ExampleNamedCommand",
    new ExampleNamedCommand(m_exampleVelocity, m_examplePosition));

// 2. Then configure the drivetrain
drivetrain.seedFieldCentric();
drivetrain.configurePathPlanner();

// 3. Then build the chooser
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Mode", autoChooser);
```

**Registration must precede `configurePathPlanner()` and `buildAutoChooser()`.** PathPlanner
resolves names when the autos load. Anything registered afterward is silently ignored — no error at
deploy, no warning at runtime, the event marker simply does nothing and the auto looks like a
mechanical failure. Names are case-sensitive and must match the `.auto` file exactly.

Compose named commands from existing teleop commands rather than reimplementing the motion. A
NamedCommand with its own copy of a mechanism's logic is a second copy to keep in sync, and it will
not stay in sync.

`getAutonomousCommand()` seeds the pose from vision before the path starts, with a timeout so a
missing tag costs half a second rather than the whole routine:

```java
return drivetrain.resetToVisionPoseCommand(m_vision)
    .withTimeout(0.5)
    .andThen(auto.asProxy());
```

`.asProxy()` lets the scheduler run the selected auto without this composition taking ownership of
its requirements — otherwise the composition would hold every subsystem the auto touches for its
entire duration.

---

## Match mode initialization

Use `RobotModeTriggers` for mode transitions. (Note: `RobotController` has no `isTeleop` /
`isAutonomous` / `isDisabled` methods — those are on `DriverStation`, and `RobotModeTriggers` is the
idiomatic form.)

```java
// Hold the drivetrain in its neutral mode while disabled.
// ignoringDisable(true) is REQUIRED — the scheduler skips commands while disabled otherwise.
final var idle = new SwerveRequest.Idle();
RobotModeTriggers.disabled().whileTrue(
    drivetrain.applyRequest(() -> idle).ignoringDisable(true)
);

// Re-home mechanisms at each mode boundary.
RobotModeTriggers.autonomous().onTrue(new ZeroExamplePositionSubsystem(m_examplePosition));
RobotModeTriggers.teleop().onTrue(new ZeroExamplePositionSubsystem(m_examplePosition));
```

Re-homing at the teleop boundary costs a second and removes a whole class of "it worked in auto"
problems — the mechanism has been moving since auto homed it.

---

## Haptic feedback

Rumble is how a driver learns something without looking away from the field. Bind it to states worth
interrupting for: a game piece acquired, a shot ready, a mechanism refusing to move, a match phase
about to change.

**Write factory functions that return commands**, so bindings stay one line:

```java
public static Command rumbleFor(CommandXboxController controller, double intensity, double seconds) {
    return Commands.startEnd(
        () -> controller.setRumble(RumbleType.kBothRumble, intensity),
        () -> controller.setRumble(RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds);
}
```

**Name the pattern for what it means, not what it does** — `rumbleGamePieceAcquired()`, not
`rumbleTwiceShort()`. When the feedback changes, the call sites still read correctly.

**Escalate intensity** for approaching deadlines rather than using one alert for everything, so the
driver can tell urgency apart without counting pulses.

**Add `.ignoringDisable(true)`** to any rumble that should fire while disabled — a pre-match ready
indicator, for instance.

Keep the *triggers* separate from the *patterns*: the season's game timing lives in the
season-specific layer, the rumble vocabulary is reusable.

---

## Epilogue logging

Epilogue generates logging code at compile time from annotated fields. It cannot serialize
everything, and a field it chokes on becomes a **compile error**, not a runtime one.

Mark anything it cannot handle:

```java
@NotLogged private PoseEstimate m_cachedFrontEstimate;
@NotLogged private long m_lastFrontNtTimestamp;
```

Typical offenders: vendor types like `PoseEstimate`, raw NT handles, cached timestamps, enum state
held for internal bookkeeping. If a build fails with an Epilogue error naming a field, that field
needs `@NotLogged`.

---

## Vision / pose estimation

Reference: `subsystems/Vision.java`, `commands/AddVisionMeasurement.java`,
`LimelightHelpers.java` (vendored — do not edit; replace wholesale on a Limelight update).

Two Limelights feed the drivetrain's pose estimator. `AddVisionMeasurement` is `Vision`'s default
command, so this runs continuously in the background.

### NT timestamp pre-check

`LimelightHelpers.getBotPoseEstimate_*()` runs `gson.fromJson()` internally. On a single-core
roboRIO that allocates heavily and can spike GC pauses. Peek at the NetworkTables server timestamp
**before** parsing — if it has not changed, the camera has no new frame and the parse can be skipped:

```java
DoubleArrayEntry frontEntry = LimelightHelpers.getLimelightDoubleArrayEntry(
    VisionConstants.LIMELIGHT_FRONT_NAME, poseEntryKey);
long frontNtTs = frontEntry.getAtomic().timestamp;

if (frontNtTs != m_lastFrontNtTimestamp
        && (now - m_lastFrontParseTime) >= VISION_MIN_PARSE_INTERVAL) {
    m_lastFrontNtTimestamp = frontNtTs;
    m_lastFrontParseTime = now;
    m_cachedFrontEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_FRONT_NAME);
}
// use m_cachedFrontEstimate — may be from this loop or an earlier one
```

Cap the parse rate at `VISION_MIN_PARSE_INTERVAL = 0.015s` (~66 Hz). A 90 fps camera does not need
to be parsed every frame to be useful.

The cached estimate, the last NT timestamp, and the last parse time all need `@NotLogged`.

### Pose rejection pipeline

Checks run cheapest-first; any failure rejects the measurement.

| Order | Check | Reason |
|---|---|---|
| 1 | `gyroRate > MAX_ANGULAR_VELOCITY` | Motion blur during fast rotation |
| 2 | `tagCount == 0` | No tags, no pose |
| 3 | `pose == null` | Defensive |
| 4 | `timestampSeconds <= 0` | Uninitialized timestamp |
| 5 | `timestamp > now + 0.5s` | Clock anomaly |
| 6 | `timestamp < now - 0.2s` | Stale |
| 7 | `timestamp <= lastAcceptedTimestamp` | Out-of-order delivery |
| 8 | `rawFiducials` null or empty | No per-tag detail |
| 9 | `rawFiducials[0].ambiguity > MAX_AMBIGUITY` | Ambiguous solve |
| 10 | `rawFiducials[0].distToCamera > MAX_TAG_DISTANCE` | Too far |
| 11 | `tagCount >= 2` | **Accept** — multi-tag, and it passed everything above |
| 12 | `avgTagDist > MAX_SINGLE_TAG_DISTANCE` | Single tag beyond reliable range |

Step 7 is the one people leave out. Feeding a Kalman filter measurements out of chronological order
makes it diverge, and the resulting pose drift looks like a hardware problem.

### Camera selection with hysteresis

With both cameras valid, naively switching to whichever is closer oscillates every loop. A small
state machine fixes it:

```
States: NONE, FRONT, BACK
- Only one camera valid          -> switch immediately
- Both valid, prefer closer avgTagDist, BUT:
    require DISTANCE_ADVANTAGE_THRESHOLD = 0.3 m of lead to switch
    enforce CAMERA_SWITCH_COOLDOWN = 0.5 s after any switch
```

### Dynamic standard deviations

Rather than fixed stddevs, scale them with measurement quality:

```java
tagCountFactor = (tagCount >= 3) ? 0.33 : (tagCount == 2) ? 0.5 : 1.0;
distanceFactor = max(0.1, min(2.0, avgTagDist));
scaledXYStdDev = max(0.1, min(2.0, baseStdDev * tagCountFactor * distanceFactor));
```

**Heading stddev is normally `9999999`** — effectively telling the estimator to ignore vision
heading, because the gyro is far more accurate for rotation. The exception: with 2+ tags, closer
than 2.5 m, ambiguity under 0.2, and the robot nearly stationary (<10 deg/s), allow a conservative
nudge (`CONSERVATIVE_HEADING_STD_DEV`, about 5 deg) to correct slow gyro drift.

### MegaTag2

Off by default (`VisionConstants.USE_MEGA_TAG_2 = false`). It fuses gyro heading into the pose
solve, which needs a well-seeded heading to help — heading error amplifies pose error at distance.
It produced poor results on 2026 hardware. **Re-evaluate each season** after a firmware or hardware
change; this is a measurement, not a permanent verdict. Validate on a real field before enabling.

### Always use the `wpiBlue` frame

Call `getBotPoseEstimate_wpiBlue()`, never `getBotPoseEstimate()`. The `wpiBlue` frame puts the
origin at the blue driver station corner with +X toward the red side. It is WPILib's standard and is
alliance-agnostic — identical on both alliances.

### `getCachedState()` in execute loops

Reading drivetrain state inside `execute()`, use `drivetrain.getCachedState()` rather than
`getState()`. The latter acquires an internal lock on every call; the former returns the snapshot
already computed in `CommandSwerveDrivetrain.periodic()` this loop.

---

## Code style conventions

**Field prefixes**

| Prefix | Use |
|---|---|
| `m_` | instance fields |
| `k` | constants (`kP`, `kCanId`, `kMaxSpeed`) |
| none | local variables |

**Command naming** — verb first, and the name should say what happens: `SetExamplePositionSubsystem`,
`RunExampleVelocitySubsystem`, `ZeroExamplePositionSubsystem`. `Set*` moves to a position, `Run*`
drives continuously, `Zero*` homes.

**Package organization**

```
frc.robot
├── constants/    Constants, RobotMap, TunableConstants
├── generated/    Tuner X output — never hand-edit
├── subsystems/
│   ├── velocity/ VelocityControlSystem + subclasses
│   └── position/ PositionControlSystem + subclasses
├── commands/
│   ├── VelocityBasedCommands/
│   └── PositionBasedCommands/
└── utils/        TelemetryManager and other shared helpers
```

Commands are grouped by control type to mirror the subsystem packages. Commands spanning several
mechanisms sit at the top of `commands/`.

**State machine enums** — declare `private enum` inside the class that owns it. A state machine
whose states are visible outside it is a state machine other code will try to drive.

**Timers** — `Timer.restart()` in `initialize()`, `hasElapsed()` to check. Never `Timer.getFPGATimestamp()`
arithmetic for command timing.

---

## AdvantageScope and simulation

### SignalLogger — `.hoot` recording

`Constants.ENABLE_SIGNAL_LOGGER` gates Phoenix's on-robot signal recorder.

**It must be `false` before any competition deploy.** The logger writes to the roboRIO's eMMC flash,
and a flash stall shows up as a main-loop overrun mid-match. Turn it on to collect SysId data or
diagnose a mechanism, then turn it back off.

### Typed struct publishers

For anything AdvantageScope should draw in 3D or on the field, publish a typed struct rather than
loose numbers:

```java
StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();
```

AdvantageScope understands these natively — no manual field mapping.

### Field2d and Mechanism2d

`Field2d` visualizes robot pose and arbitrary field objects; `Mechanism2d` draws a jointed mechanism
from its measured positions. Both are `Sendable`, so publish them once via
`TelemetryManager.putData()` — the register-once behavior exists for exactly this.

### HootAutoReplay

`Robot.java` holds a `HootAutoReplay` configured with `.withTimestampReplay()` and
`.withJoystickReplay()`. Replaying a `.hoot` in simulation reproduces a match deterministically,
including driver inputs — which is the only practical way to debug something that happened once, on
the field, with no reproduction steps.

---

## Where season-specific code goes

The template stays game-agnostic. Everything a season adds lands in predictable places:

| What | Where |
|---|---|
| New mechanisms | `subsystems/velocity/` or `subsystems/position/`, extending the base class |
| Their CAN IDs | `RobotMap.canIDs` |
| Their tuning values | `Constants`, one parameter object each |
| Their commands | `commands/VelocityBasedCommands/` or `commands/PositionBasedCommands/` |
| Autonomous actions | `commands/`, registered as NamedCommands in `RobotContainer` |
| Field coordinates, scoring locations, tag maps | a new `Constants.FieldLocations` class |
| Match phase logic, alliance-aware targeting | a new `utils/GameSpecificData.java` |
| Distance-to-setpoint interpolation | a new `utils/` calculator class |

**A note on the last two.** Game logic tends to grow tendrils into the drivetrain and into
`RobotContainer`. Keeping it behind a named class — one place that answers "what should we be aiming
at right now" — is what made the 2026 code separable at all. Alliance handling in particular should
live in exactly one method; never branch on alliance color at a call site.

---

## Template file map

What you get, and what to do with it.

**Keep and build on — the infrastructure**

| File | |
|---|---|
| `subsystems/velocity/VelocityControlSystem.java` | Velocity base class |
| `subsystems/position/PositionControlSystem.java` | Position base class |
| `utils/TelemetryManager.java` | All dashboard output |
| `constants/RobotMap.java` | Fill in your CAN IDs |
| `constants/Constants.java` | Keep the structure, replace the example instances |
| `constants/TunableConstants.java` | Keep the skeleton, replace the entries |
| `subsystems/Vision.java`, `LimelightHelpers.java`, `commands/AddVisionMeasurement.java` | Vision pipeline |
| `subsystems/CommandSwerveDrivetrain.java` | Swerve, PathPlanner wiring, SysId |
| `Robot.java`, `Main.java`, `Telemetry.java` | Framework |
| `RobotContainer.java` | Keep the skeleton, replace the bindings |

**Regenerate**

| `generated/TunerConstants.java` | Tuner X output. Regenerate for your drivetrain before it will drive correctly. |
|---|---|

**Delete once you have real mechanisms** — every `Example*` file:

```
subsystems/velocity/ExampleVelocitySubsystem.java
subsystems/position/ExamplePositionSubsystem.java
commands/VelocityBasedCommands/RunExampleVelocitySubsystem.java
commands/PositionBasedCommands/SetExamplePositionSubsystem.java
commands/PositionBasedCommands/ZeroExamplePositionSubsystem.java
commands/ExampleNamedCommand.java
```

Read them first — they are the reference for each pattern, and the comments explain the reasoning
behind choices that look arbitrary until something breaks.

---

## Adding a new motor subsystem — checklist

1. **CAN ID** into `RobotMap.canIDs`, in the right block.
2. **Constants object** into `Constants` — a `VelocityControlConstants` or
   `PositionControlConstants`. Count the arguments twice.
3. **Subsystem class** extending the base class. Constructor calls `super(canId, name)`,
   `setDefaultConstants(...)`, and `setInverted(...)`. That should be all of it.
4. **Position subsystems only:** add the unit wrapper (`setPosition`, `isAtInches`) and soft limits.
5. **Instantiate** in `RobotContainer` and bind to a button.
6. **Command** it — see the two command patterns above, or use the `add-velocity-command` /
   `add-position-command` skills.
7. **Needs homing?** Add a zeroing command and guard motion on `hasZeroed()`.
8. **Autonomous?** Register a NamedCommand — before `configurePathPlanner()`.
