---
name: add-motor-subsystem
description: Add a new motor-driven subsystem to this FRC robot codebase. Use whenever asked to add a motor, mechanism, roller, flywheel, shooter wheel, intake, elevator, arm, wrist, hood, climber, deployer, or anything else driven by a TalonFX — even if the user does not say "subsystem" or name a base class. Also use when wiring a follower motor to an existing mechanism.
---

# Add Motor Subsystem

Nearly every motor on this robot is either "spin at a speed" or "go to a position," and two base
classes cover both. A new mechanism should end up being about three lines of subsystem code — if you
are writing more than that, something belongs in the base class instead.

## Step 1: Gather requirements

Ask the user for:

1. **Subsystem name** — e.g. `Kicker`, `ArmJoint`, `ClimberWinch`
2. **Type** — velocity (RPM, Coast neutral mode) or position (inches/degrees, Brake neutral mode)?
   If unclear: does it need to *hold* somewhere, or just *spin*? Holding means position.
3. **Primary motor CAN ID**
4. **Follower motor?** If yes: its CAN ID, and whether it is *opposed* — mounted facing the other
   way across a shared shaft, so it must spin the opposite direction to help rather than fight.
5. **Inverted?** Should positive output move the mechanism the intuitive direction?
6. **Position only — `kInchesPerRotation`:** how far the mechanism travels per motor rotation. This
   is measured, not guessed. Also ask for `kGearRatio` (motor rotations per mechanism rotation),
   which drives the angle telemetry readout.
7. **Position only — travel limits:** minimum and maximum safe positions, and a setpoint tolerance.
8. **Needs zeroing?** Does it drive into a hard stop to establish its encoder reference?

## Step 2: Read these files first

Before writing anything:

- `src/main/java/frc/robot/constants/RobotMap.java` — see the numbering blocks in use
- `src/main/java/frc/robot/constants/Constants.java` — see the parameter object shape
- `src/main/java/frc/robot/RobotContainer.java` — see where subsystems get instantiated

And the reference subsystem for the type you are adding:

- Velocity: `src/main/java/frc/robot/subsystems/velocity/ExampleVelocitySubsystem.java`
- Position: `src/main/java/frc/robot/subsystems/position/ExamplePositionSubsystem.java`

The base classes themselves (`VelocityControlSystem.java`, `PositionControlSystem.java`) are worth
skimming so you know what you get for free and do not reimplement it.

## Step 3: Implement

### 1. CAN ID into `constants/RobotMap.java`

```java
public static class MySubsystem {
    public static final int MOTOR = 22;
    public static final int MOTOR_FOLLOWER = 23;  // if applicable
}
```

Pick from the right block so the Phoenix Tuner device list stays readable: drivetrain 0-8 (CANivore),
intake 20-29, handling 30-39, scoring 40-49, superstructure 50-59. Everything that is not the
drivetrain lives on the roboRIO bus.

### 2. Constants into `constants/Constants.java`

**Both are built with a builder, not a constructor.** Start with `forMotor(canId)`, chain the
`with*` calls, finish with `build()`. The method name binds each value, so call order does not
matter and a miscount is a compile error rather than a silently swapped gain.

**Velocity:**

```java
public static final VelocityControlConstants MySubsystemConstants =
    VelocityControlConstants.forMotor(RobotMap.canIDs.MySubsystem.MOTOR)
        .withPID(0.10, 0.0, 0.0, 0.12)  // kP, kI, kD, kV
        .withCurrentLimits(40.0, 60.0)  // supply amps, stator amps
        .build();
```

kP starts small and rises until it holds speed without oscillating; kI stays at 0 unless there is
steady-state error; kD is rarely useful on a velocity loop; kV around 0.12 is typical for a Kraken
or Falcon.

**Position:**

```java
public static final PositionControlConstants MySubsystemConstants =
    PositionControlConstants.forMotor(RobotMap.canIDs.MySubsystem.MOTOR)
        .withInchesPerRotation(0.8)     // measure this, do not guess
        .withGearRatio(5.0)             // telemetry only; omit on a linear mechanism
        .withMotionMagic(35.0, 70.0)    // cruise rot/s, accel rot/s^2
        .withPID(35.0, 0.0, 0.0, 0.0)   // kP, kI, kD, kV
        .withCurrentLimits(40.0, 60.0)  // supply amps, stator amps
        .build();
```

`withGearRatio` is the only optional call — it feeds the angle telemetry readout and defaults to
1.0. Every other group is required, and `build()` throws at robot boot naming any you left out.
Write ratios as `5.0`, never `5/1`: that is an int expression and truncates silently.

**Position mechanisms also need travel limits.** Keep them beside the constants so the clamp inside
the subsystem and the setpoints commands ask for cannot drift apart:

```java
public static final class MySubsystemPositions {
    public static final double MIN_POSITION = 0.0;
    public static final double MAX_POSITION = 12.0;
    public static final double STOWED = 0.0;
    public static final double DEPLOYED = 11.5;
    public static final double POSITION_TOLERANCE = 0.25;
}
```

### 3. Create the subsystem class

**Velocity** — `src/main/java/frc/robot/subsystems/velocity/MySubsystem.java`

```java
package frc.robot.subsystems.velocity;

import frc.robot.constants.Constants;

public class MySubsystem extends VelocityControlSystem {
    public MySubsystem() {
        super(Constants.MySubsystemConstants.kCanId, "MySubsystem");
        setDefaultConstants(Constants.MySubsystemConstants);
        setInverted(false);
    }
}
```

That is the whole file. PID configuration, current limits, StatusSignal caching, batched CAN
refreshes, and telemetry all come from the base class.

**Position** — `src/main/java/frc/robot/subsystems/position/MySubsystem.java`

A position subsystem adds one thing: the **unit boundary**. The base class works in motor rotations;
commands and drivers work in inches or degrees. Convert here, in one place.

```java
package frc.robot.subsystems.position;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.MySubsystemPositions;

public class MySubsystem extends PositionControlSystem {
    public MySubsystem() {
        super(Constants.MySubsystemConstants.kCanId, "MySubsystem");
        setDefaultConstants(Constants.MySubsystemConstants);
        setInverted(false);
    }

    public void setPosition(double inches) {
        double clamped = MathUtil.clamp(inches,
            MySubsystemPositions.MIN_POSITION, MySubsystemPositions.MAX_POSITION);
        setRotation(inchesToRotations(clamped));
    }

    public double getPositionInches() {
        return rotationsToInches(m_positionSignal.getValueAsDouble());
    }

    public boolean isAtInches(double targetInches, double toleranceInches) {
        return Math.abs(getPositionInches() - targetInches) < toleranceInches;
    }
}
```

**Name the setpoint check `isAtInches`, not `isAtSetpoint`.** The base class already has
`isAtSetpoint(double, double)` comparing *rotations*. Declaring an inches version at that same
signature overrides it silently — any caller holding a base-class reference then gets inches
semantics while reading code that says rotations, and nothing at the call site reveals it.

**Followers** do not get their own class. Give the follower a CAN ID and its own thin subsystem, then
call `followMotor()` once — from the follower's constructor, or from `RobotContainer` after both
exist:

```java
m_myFollower.followMotor(m_mySubsystem, /* opposed = */ true);
```

A following motor ignores `setSpeed()` entirely; it mirrors the leader's output directly, which is
what you want for a two-motor gearbox where the motors are mechanically coupled.

### 4. Wire it into `RobotContainer.java`

```java
private final MySubsystem m_mySubsystem = new MySubsystem();
```

Then bind it in `configureBindings()`, under the operator controller section — mechanism controls
belong there so the driver's controller stays dedicated to driving.

## Critical rules

- **Never command a position subsystem to `0` in `end()`.** Motion Magic holds the last setpoint, so
  an empty `end()` is what keeps an arm up. Commanding zero drives a gravity-loaded mechanism to the
  floor at full authority; `stopMotor()` drops it.
- **Inversion goes in the constructor**, never inside a command. Fix it in a command and every other
  command driving that motor has to remember to do the same.
- **`setSpeed()` takes RPM.** The base class converts to RPS. `getSpeedRPM()` and `getSpeedRPS()` are
  both available and named for their unit.
- **Do not reconfigure PIDF in the subsystem.** It all goes through `setDefaultConstants()`.
- **If a stall threshold is involved, check it against `kStatorCurrentLimit`.** A threshold at or
  above the limit can never be reached, because the limit caps current first.

## After completion

- Needs encoder calibration? Use the **add-zeroing-command** skill.
- Needs a command to drive it? Use **add-velocity-command** or **add-position-command**.
- Needs to run in autonomous? Use **register-named-command**.
