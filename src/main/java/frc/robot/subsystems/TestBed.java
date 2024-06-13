package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team5430.util.CollisionDetection;
import com.team5430.util.SwerveModule;
import com.team5430.util.SwerveModuleGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TestBed extends SubsystemBase {

  public TestBed() {}

  // init objeccts
  public AHRS gyro = new AHRS(Port.kMXP);

  public CollisionDetection mfeedback = new CollisionDetection(gyro);

  // swerve CANids; 0 through 7, even being drive motor, and odd being angle motor, following
  // alphabetically,
  // Module A -> Module B -> Module C
  SwerveModuleGroup DriveTrain =
      new SwerveModuleGroup(
          new SwerveModule(0, 1, "Module A"),
          new SwerveModule(2, 3, "Module B"),
          new SwerveModule(4, 5, "Module C"),
          new SwerveModule(6, 7, "Module D"));

  public double lastAngle = 0;

  public void motorConfig() {
    DriveTrain.addTab();
  }

    // **the wheel will go to the position that is greater than 0.2, otherwise stop power when less
  // than or equal to*/
  public double deadzone(double angle, double power) {
    // If the input given is less than 0.3 the rotation will reset to 0
    if (power < -0.3) {
      lastAngle = angle;
      if (power > -0.3) {
        return angle;
      }
    }
    return lastAngle;
  }

  // setAngle will set the directional angle
  public void drive(
      double wantedAngle, double throttle, double z, double gyroscope, double breaking) {
    DriveTrain.Drive(deadzone(wantedAngle, -throttle), throttle, z, gyroscope, breaking);
  }

  @Override
  public void periodic() {

    // collision detection
    if (mfeedback.CollisionDetected() == true) {
      RobotContainer.driverController.setRumble(true);
    } else {
      RobotContainer.driverController.setRumble(false);
    }

    SmartDashboard.putBoolean("Collision detection", mfeedback.CollisionDetected());
    SmartDashboard.putNumber(
        "degrees", RobotContainer.driverController.getLeftStickDirrectionDegrees());
    SmartDashboard.putNumber("magnitude", RobotContainer.driverController.getLeftMagnitude());
  }
}
