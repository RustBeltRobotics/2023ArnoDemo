package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This command is used to drive the robot with a coordinate system that is
 * relative to the robot, not the field.
 */
public class RobotOrientedDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final IntSupplier povSupplier;

    public RobotOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            IntSupplier povSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.povSupplier = povSupplier;

        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input x, y, and rotation velocities to the drivetrain's drive method
     * as a ChassisSpeed object.
     */
    @Override
    public void execute() {
        int pov = povSupplier.getAsInt();
        if (pov == -1) {
            drivetrain.drive(new ChassisSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    rotationSupplier.getAsDouble()));
        } else {
            drivetrain.drive(new ChassisSpeeds(
                    Math.cos(Math.toRadians(pov)),
                    Math.sin(Math.toRadians(pov - 180)),
                    0.));
        }
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
