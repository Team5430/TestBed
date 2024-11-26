package frc.robot.subsystems;

import com.team5430.swerve.SwerveModuleConstants;
import com.team5430.vision.LimeLight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  // TODO: integrate vision with pose estimator

  // init camera
  protected LimeLight camera =
      new LimeLight(Constants.VisionConstants.CameraName, Constants.VisionConstants.CameraToRobot);

  // init led control
  protected LimeLight.LEDs led_control = camera.new LEDs();

  protected static Vision mInstance = new Vision();

  public static Vision getInstance(){
    return mInstance;
  }

  public Vision() {}

  // add triggers
  public Trigger TagInRange() {
    return camera.AprilTagDetected;
  }

  //Megatag2 getter for pose !!!
  public Pose2d getPose2d(double robotAngle){
    return camera.getPose2d(robotAngle);
  }

  /**timestamp of {@code getPose2d}*/
  public double getPoseTimestamp(){
    return camera.poseEstimateTimestamp();
  }

  // math w/ camera
  public double proportionalAim() {
    return camera.ProportinalAim(new SwerveModuleConstants().MAX_OMEGA_RADIANS);
  }

  public double proportionalRange() {
    return camera.ProportinalRange(new SwerveModuleConstants().MAX_VELOCITY_MPS);
  }

  @Override
  public void periodic() {
  //update camera results 
    camera.updateResults();
    //TODO: modify periodic loop time for vison; add practical functionality to {@code Limelight.LEDs}
  }

}
