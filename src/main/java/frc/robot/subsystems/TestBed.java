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

  // init objects
  public AHRS gyro = new AHRS(Port.kMXP);

  public CollisionDetection mfeedback = new CollisionDetection();

  // TODO change motor CANids irl

  // swerve CANids; 0 through 10, following a pattern of angle motor, drive motor, and CANcoder id,
  // alphabetically
  // Module A [0, 1 (2 CANcoder set based on those ids) -> Module B [3, 4, 5,] Module C
  SwerveModuleGroup DriveTrain =
      new SwerveModuleGroup(
          new SwerveModule(0, 1, "Module A"),
          new SwerveModule(3, 4, "Module B"),
          new SwerveModule(6, 7, "Module C"),
          new SwerveModule(9, 10, "Module D"));

  public void motorConfig() {
    DriveTrain.addTab();
  }

  public void ControllerVibration() {
    RobotContainer.driverController.setRumble(mfeedback.CollisionDetected());
  }

  // setAngle will set the directional angle
  public void drive(
      double wantedAngle, double throttle, double turning, double gyroscope, double breaking) {
    DriveTrain.Drive(wantedAngle, throttle, turning, gyroscope, breaking);
  }

  @Override
  public void periodic() {

    // collision detection
    SmartDashboard.putBoolean("Collision detection", mfeedback.CollisionDetected());

    SmartDashboard.putNumber(
        "degrees", RobotContainer.driverController.getLeftStickDirectionDegrees());
    SmartDashboard.putNumber("magnitude", RobotContainer.driverController.getLeftMagnitude());
  }
}
