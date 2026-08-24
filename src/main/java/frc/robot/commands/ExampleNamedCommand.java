package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PositionBasedCommands.SetExamplePositionSubsystem;
import frc.robot.commands.VelocityBasedCommands.RunExampleVelocitySubsystem;
import frc.robot.constants.Constants.ExamplePositionSubsystemPositions;
import frc.robot.subsystems.position.ExamplePositionSubsystem;
import frc.robot.subsystems.velocity.ExampleVelocitySubsystem;

/**
 * Reference PathPlanner NamedCommand: deploy, run, retract.
 *
 * <p>This is what an autonomous action looks like end to end. Registered under a name in
 * {@code RobotContainer}, it can then be dropped onto an event marker in the PathPlanner GUI and
 * triggered partway along a path.
 *
 * <p><b>It composes existing commands rather than reimplementing them.</b> That is the habit worth
 * copying: every behavior here already exists as a teleop command, so the autonomous version cannot
 * drift away from what the drivers actually tested. A NamedCommand that reimplements a mechanism's
 * motion is a second copy of that logic to keep in sync, and it will not stay in sync.
 *
 * <p>Registration lives in the {@code RobotContainer} constructor and <b>must</b> happen before
 * {@code configurePathPlanner()} and {@code AutoBuilder.buildAutoChooser()}. PathPlanner resolves
 * names when the autos load; anything registered afterward is silently ignored, with no error at
 * deploy and no warning at runtime -- the marker simply does nothing and the auto looks like a
 * mechanical failure.
 *
 * <p>Delete this file once you have real autonomous actions.
 */
public class ExampleNamedCommand extends SequentialCommandGroup {

    /** How long to run the velocity mechanism once deployed. */
    private static final double RUN_SECONDS = 2.0;

    public ExampleNamedCommand(ExampleVelocitySubsystem velocitySubsystem,
                               ExamplePositionSubsystem positionSubsystem) {
        addCommands(
            // Deploy. Finishes when the mechanism arrives, or on its own timeout -- either way the
            // sequence keeps moving, which is what you want when the clock is running.
            new SetExamplePositionSubsystem(
                positionSubsystem, ExamplePositionSubsystemPositions.DEPLOYED),

            // Run for a fixed duration. RunExampleVelocitySubsystem never finishes by itself, so
            // the timeout is what ends it.
            new RunExampleVelocitySubsystem(velocitySubsystem, 2000.0).withTimeout(RUN_SECONDS),

            // Retract before driving on.
            new SetExamplePositionSubsystem(
                positionSubsystem, ExamplePositionSubsystemPositions.STOWED),

            // Commands.print() output shows up in the driver station log with a timestamp, which
            // makes it easy to confirm afterward that a marker actually fired.
            Commands.print("ExampleNamedCommand complete")
        );

        // Note: no addRequirements() here. A SequentialCommandGroup inherits the requirements of
        // every command inside it, so declaring them again would be redundant.
    }
}
