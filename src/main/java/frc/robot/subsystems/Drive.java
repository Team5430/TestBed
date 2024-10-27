package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team5430.util.SwerveModuleConstants;
import com.team5430.util.SwerveModuleGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

//init 
  private final StructPublisher<Rotation2d> publisher;

//Swerve Config
  private SwerveModuleConstants mConfig = new SwerveModuleConstants();

//Swerve DriveTrain
  protected SwerveModuleGroup DriveTrain = new SwerveModuleGroup(4, mConfig);

//gyro
  public AHRS mGyro = new AHRS(Port.kMXP);

  public Drive() {

    // data logging
    publisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/Rotation2d", Rotation2d.struct)
            .publish();

    ResetHeading();
  }

  
//init robot state
  private SwerveDriveOdometry m_Odometry =
      new SwerveDriveOdometry(mConfig.Kinematics, getRotation2d(), DriveTrain.getPositions(true));

  // options to drive
  public void Drive(ChassisSpeeds input, Rotation2d robotAngle, boolean isFieldCentric) {

    if (isFieldCentric) {

      DriveTrain.FieldCentricDrive(input, robotAngle);
    } else {
      DriveTrain.RobotRelativeDrive(input);
    }
  }

  public void ResetHeading() {
    mGyro.reset();
  }

  //get position
  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  //reset position
  public void resetPose() {
    m_Odometry.resetPosition(getRotation2d(), DriveTrain.getPositions(true), getPose());
  }

  public Rotation2d getRotation2d() {
    return mGyro.getRotation2d();
  }

  // loops stuff
  @Override
  public void periodic() {

    publisher.set(getRotation2d());
    DriveTrain.publishData();
  }

  public Command DriveToDistance(double Distance) {
    return new InstantCommand(() -> DriveTrain.DriveToDistance(Distance), this);
  }
}
