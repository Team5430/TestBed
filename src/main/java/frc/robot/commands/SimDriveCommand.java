package frc.robot.commands;

import com.team5430.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;
import java.util.function.DoubleSupplier;

public class SimDriveCommand extends Command {

    // Subsystem to require
    private final Drive mDrive;

    // Double suppliers for human inputs
    private final DoubleSupplier xTranslation, yTranslation, rTranslation;

    // Toggle per preference: true for field-centric drive, false for robot-centric
    private static final Boolean FIELD_CENTRIC = false;

    // Constants for swerve
    private final SwerveModuleConstants constants = new SwerveModuleConstants();

    /**
     * Command for driving the simulated DriveTrain
     * Same as DriveCommand, without any modifiers to inputs
     *
     * @param X x translation control
     * @param Y y translation control
     * @param Rotation rotational control
     * @param subsystem drive subsystem that is required
     */
    public SimDriveCommand(
            DoubleSupplier X,
            DoubleSupplier Y,
            DoubleSupplier Rotation,
            Drive subsystem) {
        this.xTranslation = X;
        this.yTranslation = Y;
        this.rTranslation = Rotation;
        this.mDrive = subsystem;

        // Require Drive subsystem
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        // Get inputs
        double x = xTranslation.getAsDouble();
        double y = yTranslation.getAsDouble();
        double rotation = rTranslation.getAsDouble();

        // Setup for type of drivestyle
        boolean driveStyleToggle = FIELD_CENTRIC;
        Rotation2d robotAngle = mDrive.getRotation2d();

        // Apply inputs
        ChassisSpeeds inputs = new ChassisSpeeds(
               - x  * constants.MAX_VELOCITY_MPS,
                y  * constants.MAX_VELOCITY_MPS,
                rotation * constants.MAX_OMEGA_RADIANS
        );

        // Drive with inputs
        mDrive.control(inputs, robotAngle, driveStyleToggle);
    }

    // Stop the drivetrain
    @Override
    public void end(boolean interrupted) {
        mDrive.Stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}