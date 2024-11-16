package frc.robot.subsystems;

import com.team5430.swerve.SwerveModuleConstants;
import com.team5430.vision.LimeLight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  // TODO: integrate vision with pose estimators in @Drive

  // init camera
  protected LimeLight camera =
      new LimeLight(Constants.VisionConstants.CameraName, Constants.VisionConstants.CameraToRobot);

  // init led control
  protected LimeLight.LEDs led_control = camera.new LEDs();

  public Vision() {}

  // add triggers
  public Trigger TagInRange() {
    return camera.AprilTagDetected;
  }

  // math w/ camera
  public double proportionalAim() {
    return camera.ProportinalAim(new SwerveModuleConstants().MAX_OMEGA_RADIANS);
  }

  public double proportionalRange() {
    return camera.ProportinalRange(new SwerveModuleConstants().MAX_VELOCITY_MPS);
  }
}
