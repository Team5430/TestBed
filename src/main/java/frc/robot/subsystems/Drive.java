package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team5430.swerve.SwerveModuleConstants;
import com.team5430.swerve.SwerveModuleGroup;
import com.team5430.util.booleans;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

  // init
  private final StructPublisher<Rotation2d> publisher;

  // Swerve Config
  private final SwerveModuleConstants mConfig = new SwerveModuleConstants();

  // Swerve DriveTrain
  protected SwerveModuleGroup DriveTrain = new SwerveModuleGroup(4, mConfig);

  // gyro
  public AHRS mGyro = new AHRS(Port.kMXP);


  HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
          5,
          4,
          new ReplanningConfig()
  );

  public Drive() {

    // data logging
    publisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/Rotation2d", Rotation2d.struct)
            .publish();

    //make sure gyro is calibrated
    ResetHeading();
    //configure for auton
    configurePathPlanner();

  }

  // init robot state
  protected SwerveDriveOdometry Odometry =
      new SwerveDriveOdometry(mConfig.Kinematics, getRotation2d(), DriveTrain.getPositions(true));

  // options to drive
  public void control(ChassisSpeeds input, Rotation2d robotAngle, boolean isFieldCentric) {

    if (isFieldCentric) {

      DriveTrain.FieldCentricDrive(input, robotAngle);
    } else {
      DriveTrain.RobotRelativeDrive(input);
    }
  }

  //configure robot control during auton
  private void configurePathPlanner(){

     //configure robot driving for auton
    AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            DriveTrain::getCurrentSpeeds,
            DriveTrain::RobotRelativeDrive,
            config,
            booleans.isBlue(),
            this
    );
  }

  //Stops the DriveTrain
  public void Stop(){
    DriveTrain.Stop();
  }

  //zeros the gyro
  public void ResetHeading() {
    mGyro.reset();
  }

  // get position
  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  // reset position
  public void resetPose(Pose2d pose) {
    Odometry.resetPosition(getRotation2d(), DriveTrain.getPositions(true), pose);
  }

  //get heading as a Rotation2d
  public Rotation2d getRotation2d() {
    return mGyro.getRotation2d();
  }

  // loops stuff
  @Override
  public void periodic() {
    
    publisher.set(getRotation2d());
    DriveTrain.publishData();
  }

}
