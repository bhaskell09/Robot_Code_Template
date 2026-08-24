package frc.robot.constants;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Values you can change from the dashboard while the robot is running, without a redeploy.
 *
 * <p>These live in the "Tuning" NetworkTable. Open it in Shuffleboard, Elastic, or AdvantageScope,
 * drag a number, and the robot picks it up on the next loop -- which is what makes a tuning session
 * take ten minutes instead of an afternoon of build-and-deploy cycles.
 *
 * <p><b>How the value actually reaches the robot:</b> a {@code DoubleEntry} reads live from
 * NetworkTables every time you call {@code .get()}. Nothing pushes or polls on a timer, and
 * {@code RobotContainer.periodic()} does not need to do anything for this to work. What matters is
 * <i>where</i> you call the getter: read it inside a command's {@code execute()} and edits take
 * effect immediately; read it once in a constructor and you have frozen the value at robot boot.
 *
 * <p><b>Once a value is tuned, move it into {@link Constants} and delete the entry here.</b> A
 * tunable that outlives its tuning session is a value nobody can find in the code and that silently
 * resets to its default if the dashboard layout is lost.
 *
 * <p><b>Adding one:</b> declare the entry in section 1, add a getter in section 2, add the matching
 * {@code setDefault} in section 3 -- and make sure the two defaults agree. The 2026 code specified
 * them in both places with different numbers, so what you got depended on whether the NT topic had
 * been published yet. Keeping them equal removes the ambiguity entirely.
 */
public class TunableConstants {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Tuning");

    // =============================================================
    // 1. DEFINE YOUR TUNABLE VARIABLES HERE
    // =============================================================

    // Naming convention: "Mechanism/Value". The slash groups them in the dashboard tree.
    private static final DoubleEntry exampleVelocityRPM =
        table.getDoubleTopic("ExampleVelocitySubsystem/RPM").getEntry(2000.0);

    private static final DoubleEntry examplePositionInches =
        table.getDoubleTopic("ExamplePositionSubsystem/Inches").getEntry(0.0);

    // Feature toggles work the same way and are handy for A/B testing a change on the field.
    private static final BooleanEntry useGainScheduling =
        table.getBooleanTopic("Drivetrain/UseGainScheduling").getEntry(true);

    // =============================================================
    // 2. STATIC GETTERS (call these from anywhere)
    // =============================================================

    public static double getExampleVelocityRPM() {
        return exampleVelocityRPM.get();
    }

    public static double getExamplePositionInches() {
        return examplePositionInches.get();
    }

    public static boolean useGainScheduling() {
        return useGainScheduling.get();
    }

    // =============================================================
    // 3. INITIALIZATION
    //
    // Called once from the RobotContainer constructor. Publishes each topic so
    // the keys appear in the dashboard tree before anyone touches them, and
    // gives every entry a starting value.
    //
    // Each default must match the value passed to getEntry() above.
    // =============================================================

    public static void initializeDefaults() {
        exampleVelocityRPM.setDefault(2000.0);
        examplePositionInches.setDefault(0.0);
        useGainScheduling.setDefault(true);
    }

    private TunableConstants() {}
}
