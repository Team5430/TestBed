package frc.robot.commands;

import com.team5430.util.MathHelpers;
import com.team5430.util.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

  Drive mDrive;

  DoubleSupplier xTranslation, yTranslation, rTranslation, ThrottleBreaker;

  Boolean FIELD_CENTRIC = false;

  SwerveModuleConstants constants;

  public DriveCommand(
      DoubleSupplier X,
      DoubleSupplier Y,
      DoubleSupplier Rotation,
      DoubleSupplier breaking,
      Drive subsystem) {
    // get inputs
    xTranslation = X;
    yTranslation = Y;
    rTranslation = Rotation;
    ThrottleBreaker = breaking;
    mDrive = subsystem;
    // require Drive subsystem
    addRequirements(subsystem);
  }

  @Override
  public void execute() {

    // get inputs
    double x = xTranslation.getAsDouble();
    double y = yTranslation.getAsDouble();
    double rotation = rTranslation.getAsDouble();
    double breaking = MathHelpers.VariableSpeedDecline(ThrottleBreaker.getAsDouble());

    // setup for type of drivestyle
    boolean DriveStyleToggle = FIELD_CENTRIC;
    Rotation2d RobotAngle = mDrive.getRotation2d();

    // apply inputts
    ChassisSpeeds Inputs = new ChassisSpeeds(
            -x * breaking * constants.MAX_VELOCITY_MPS,
             y * breaking * constants.MAX_VELOCITY_MPS,
            rotation * constants.MAX_OMEGA_RADIANS);

    // drive with inputs
    mDrive.control(Inputs, RobotAngle, DriveStyleToggle);
  }

  @Override
  public void end(boolean interupted) {
    mDrive.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
