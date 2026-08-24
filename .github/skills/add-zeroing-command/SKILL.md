---
name: add-zeroing-command
description: Create a hard-stop zeroing command for a position-controlled mechanism. Use whenever asked to add zeroing, homing, calibration, or an encoder reset for a mechanism that drives into a physical stop — an arm, elevator, wrist, hood, deployer, or climber. Also use when a mechanism does not know where it is at power-on, when setpoints are off by a constant offset, or when someone asks how to find a mechanism's reference position.
---

# Add Zeroing Command

Zeroing drives a mechanism into a physical hard stop at low voltage, detects the stall via stator current, then resets the encoder to a known position. This is the standard calibration method for mechanisms without absolute encoders.

## Step 1: Gather Requirements

Ask the user for:

1. **Mechanism name** (e.g. `ArmJoint`, `Elevator`, `Deployer`) — the command will be named `ZeroXxx`
2. **Zeroing direction** — which physical direction reaches the hard stop?
3. **Retract voltage (V)** — briefly clears the stop before approach (typically −1 to −3 V)
4. **Approach voltage (V)** — slow drive into the stop (typically 1 to 3 V)
5. **Stall current threshold (A)** — indicates hard stop contact (typically 20–40 A)
6. **Stall debounce time (s)** — how long current must stay high (typically 0.1–0.3 s)
7. **Safety timeout (s)** — max time before giving up (typically 3–5 s)
8. **Encoder zero point** — does the hard stop = 0 inches, or is there an offset?

## Step 2: Read Reference Files First

Before writing anything, read:
- `src/main/java/frc/robot/commands/PositionBasedCommands/ZeroExamplePositionSubsystem.java` — reference implementation
- `src/main/java/frc/robot/subsystems/position/ExamplePositionSubsystem.java` — reference subsystem
- `src/main/java/frc/robot/subsystems/position/PositionControlSystem.java` — the zeroing API lives in the base class
- `src/main/java/frc/robot/constants/Constants.java` — where the zeroing constants go

## Step 3: Add Zeroing Constants to `Constants.java`

```java
public static class MyMechanismZeroingConstants {
    public static final double kRetractVolts       = -2.0;  // volts
    public static final double kApproachVolts      =  2.0;  // volts
    public static final double kStallCurrentAmps   = 25.0;  // A
    public static final double kDebounceSeconds    =  0.15; // s
    public static final double kTimeoutSeconds     =  5.0;  // s
    public static final double kRetractTimeSeconds =  0.2;  // s
    public static final double kHardStopInches     =  0.0;  // mechanism position AT the stop
}
```

## Step 4: Verify the Subsystem Exposes the Zeroing API

Most of this comes free from `PositionControlSystem`:

```java
public void   markZeroed()                              // sets the persistent flag
public void   clearZeroed()                             // clears it
public boolean hasZeroed()                              // reads it
public void   setEncoderToHardStop(double inches)       // writes the stop's position, marks zeroed
public double getStatorCurrent()                        // cached — no extra CAN read
public void   setVoltage(double volts)                  // open-loop, bypasses Motion Magic
```

The only thing a concrete subsystem usually adds is a named wrapper for the open-loop output, so the
command reads clearly:

```java
public void setZeroingOutput(double volts) { setVoltage(volts); }
```

Note `setEncoderToHardStop(inches)` takes the mechanism position **of the stop itself** and calls
`markZeroed()` for you. That is often 0, but a mechanism homing against its fully-extended stop
should pass its extended travel instead.

## Step 5: Generate the Zeroing Command

Create `src/main/java/frc/robot/commands/PositionBasedCommands/ZeroMyMechanism.java`:

```java
package frc.robot.commands.PositionBasedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.MyMechanismZeroingConstants;
import frc.robot.subsystems.position.MyMechanism;
import frc.robot.utils.TelemetryManager;

public class ZeroMyMechanism extends Command {

    private enum Phase { RETRACTING, EXTENDING }

    private final MyMechanism m_mechanism;
    private Phase m_phase;
    private boolean m_stallDetected = false;
    private double m_stallStartTime = 0.0;
    private final Timer m_retractTimer = new Timer();
    private final Timer m_safetyTimer  = new Timer();
    private boolean m_alreadyZeroed = false;

    public ZeroMyMechanism(MyMechanism mechanism) {
        m_mechanism = mechanism;
        // Requirements DO belong here: zeroing owns the mechanism for its duration, and a
        // position command running concurrently would fight it for control of the motor.
        addRequirements(m_mechanism);
    }

    @Override
    public void initialize() {
        if (m_mechanism.hasZeroed()) { m_alreadyZeroed = true; return; }
        m_alreadyZeroed = false;
        m_stallDetected = false;
        m_phase = Phase.RETRACTING;
        m_retractTimer.reset(); m_retractTimer.start();
        m_safetyTimer.reset();  m_safetyTimer.start();
        m_mechanism.setZeroingOutput(MyMechanismZeroingConstants.kRetractVolts);
        TelemetryManager.putString("ZeroMyMechanism/status", "RETRACTING");
    }

    @Override
    public void execute() {
        if (m_alreadyZeroed) return;
        switch (m_phase) {
            case RETRACTING:
                if (m_retractTimer.hasElapsed(MyMechanismZeroingConstants.kRetractTimeSeconds)) {
                    m_phase = Phase.EXTENDING;
                    m_mechanism.setZeroingOutput(MyMechanismZeroingConstants.kApproachVolts);
                    TelemetryManager.putString("ZeroMyMechanism/status", "EXTENDING");
                }
                break;
            case EXTENDING:
                if (m_mechanism.getStatorCurrent() > MyMechanismZeroingConstants.kStallCurrentAmps) {
                    if (!m_stallDetected) {
                        m_stallDetected = true;
                        m_stallStartTime = Timer.getFPGATimestamp();
                        TelemetryManager.putString("ZeroMyMechanism/status", "STALL DETECTED");
                    }
                } else {
                    m_stallDetected = false; // transient spike — reset debounce
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (m_alreadyZeroed) return true;
        boolean stallComplete = m_stallDetected
            && (Timer.getFPGATimestamp() - m_stallStartTime) >= MyMechanismZeroingConstants.kDebounceSeconds;
        return stallComplete || m_safetyTimer.hasElapsed(MyMechanismZeroingConstants.kTimeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        m_mechanism.stopMotor();
        if (m_alreadyZeroed) return;
        boolean timedOut = m_safetyTimer.hasElapsed(MyMechanismZeroingConstants.kTimeoutSeconds);
        if (m_stallDetected && !timedOut) {
            // setEncoderToHardStop() marks the mechanism zeroed as part of writing the position.
            m_mechanism.setEncoderToHardStop(MyMechanismZeroingConstants.kHardStopInches);
            TelemetryManager.putString("ZeroMyMechanism/status", "SUCCESS");
        } else {
            TelemetryManager.putString("ZeroMyMechanism/status", "FAILED — timeout");
            // Do NOT call markZeroed() — encoder has not been reset
        }
    }
}
```

## Critical Rules

- **Never call `markZeroed()` on timeout** — the encoder position is unknown.
- **`hasZeroed()` is persistent** — survives mode transitions. Cleared only by `clearZeroed()` (called at autonomous start for mechanisms that re-zero in auto).
- **Guard all position commands** — `setPosition()` and any command driving it should refuse to move while `!hasZeroed()`. Commanding a position off an untrusted encoder sends the mechanism somewhere nobody predicted, at full authority.
- **Retract first, then approach** — consistent approach direction prevents false stall readings.
- **Debounce prevents false positives** — raw stator current spikes on direction changes and at any sticky spot in the travel. Without debouncing, the first of those zeroes the mechanism mid-range.
- **Check the stall threshold against `kStatorCurrentLimit`.** If the threshold is at or above the limit, the limit caps current before the detector can ever fire and homing silently never completes. This shipped once: a 7.5 A limit against a 9.0 A threshold.

## After Completion

If this command will be used in PathPlanner autos, use the **register-named-command** skill to register it.
