package frc.robot.commands;

import com.team5430.util.MathHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

  Drive mDrive;

  DoubleSupplier xTranslation, yTranslation, rTranslation, ThrottleBreaker;

  Boolean FIELD_CENTRIC = false;

  public DriveCommand(
      DoubleSupplier X,
      DoubleSupplier Y,
      DoubleSupplier Rotation,
      DoubleSupplier breaking,
      Drive subsystem) {
  //get inputs
    xTranslation = X;
    yTranslation = Y;
    rTranslation = Rotation;
    ThrottleBreaker = breaking;
    mDrive = subsystem;
  // require Drive subsystem
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
  //get inputs
    double x = xTranslation.getAsDouble();
    double y = yTranslation.getAsDouble();
    double rotation = rTranslation.getAsDouble();
    double breaking = MathHelpers.VariableSpeedDecline(ThrottleBreaker.getAsDouble());

  //setup for type of drivestyle
    boolean DriveStyleToggle = FIELD_CENTRIC;
    Rotation2d RobotAngle = mDrive.getRotation2d();

  //apply inputts
    ChassisSpeeds Inputs = new ChassisSpeeds(x * breaking, y * breaking, rotation);

  //drive with inputs
    mDrive.Drive(Inputs, RobotAngle, DriveStyleToggle);
  }

  @Override
  public void end(boolean interupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
