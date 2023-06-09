package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends CommandBase {
    private final Intake intake;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier inSpeedSupplier;
    private final DoubleSupplier outSpeedSupplier;

    public DefaultIntakeCommand(Intake intake,
            DoubleSupplier inSpeedSupplier,
            DoubleSupplier outSpeedSupplier) {
        this.intake = intake;
        this.inSpeedSupplier = inSpeedSupplier;
        this.outSpeedSupplier = outSpeedSupplier;

        // Command requires the intake subsystem
        addRequirements(intake);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input speeds to the appropriate intake/outtake method
=     */
    @Override
    public void execute() {
        double inSpeed = inSpeedSupplier.getAsDouble();
        double outSpeed = outSpeedSupplier.getAsDouble();
        double netSpeed = Math.abs(inSpeed - outSpeed);

        if (inSpeed > outSpeed) {
            intake.runIntake(netSpeed, true);
        } else {
            intake.runIntake(netSpeed, false);
        }
    }

    /** When the intake method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

}
