# Pre-Match System Checklist

Run this in the pit before every match. It is deliberately mechanical — the point is to catch the
thing nobody thought to check, and that only works if you actually walk the list rather than
remembering it.

**Update the mechanism sections each season.** Replace the placeholders below with this year's
mechanisms and the buttons that actually drive them, and do it before the first competition rather
than at it. A checklist describing last year's robot is worse than none, because people tick it.

---

## Drivetrain

- [ ] Move left stick forward — verify correct direction
- [ ] Move left stick backward — verify correct direction
- [ ] Move left stick left — verify correct direction
- [ ] Move left stick right — verify correct direction
- [ ] Move right stick left (rotate CCW) — verify correct direction
- [ ] Move right stick right (rotate CW) — verify correct direction
- [ ] All four wheels respond equally
- [ ] Wheels are clean and free of debris
- [ ] All steer angles are zeroed
- [ ] Gyro has been zeroed since the driver took control

## Vision

- [ ] Both Limelights powered and streaming
- [ ] Pose estimate on the dashboard looks sane while the robot sits still
- [ ] Pose does not jump when the robot rotates in place

## [Mechanism 1 — rename me]

- [ ] Press **[button]** — verify correct direction
- [ ] Verify it runs at the correct speed
- [ ] No visible damage from previous matches
- [ ] Nothing stuck or jammed

## [Mechanism 2 — rename me]

- [ ] Press **[button]** to home the mechanism — verify it finds its stop and stops
- [ ] Press **[button]** to deploy — verify it travels smoothly and holds position
- [ ] Press **[button]** to retract — verify it travels smoothly and holds position
- [ ] Mechanism holds position when the button is released (does not sag or drop)
- [ ] No visible damage from previous matches

## Full cycle

- [ ] Run the complete scoring sequence end to end
- [ ] Everything moves smoothly with no stalls or jams
- [ ] Nothing is left in a position that could catch on a field element

## Autonomous

- [ ] Correct auto selected in the chooser
- [ ] Robot is placed on its starting position and heading for that auto

## General

- [ ] Battery voltage is healthy (≥ 12.2 V)
- [ ] roboRIO is running (status lights green)
- [ ] Wireless radio is functioning
- [ ] Driver Station connects with no errors
- [ ] Robot is stable on its bumpers
- [ ] All cables secured — no snagging risk
- [ ] All mechanisms stowed inside frame perimeter
- [ ] No loose parts that could fall off
- [ ] Every mechanism that homes has been homed since the last power cycle

## Competition build

Verify once per event, not per match:

- [ ] `Constants.ENABLE_SIGNAL_LOGGER` is `false`
- [ ] `TelemetryManager.setEnabled(false)` in the `RobotContainer` constructor
