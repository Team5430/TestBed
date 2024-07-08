package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team5430.util.CollisionDetection;
import com.team5430.util.SwerveModuleConstants;
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

  private final SwerveModuleConstants SwerveConfig = new SwerveModuleConstants();

  // TODO change motor CANids irl

  public SwerveModuleGroup DriveTrain = new SwerveModuleGroup(4, SwerveConfig);

  public void motorConfig() {
    SwerveConfig.SteeringGearRatio = 1;
    SwerveConfig.ThrottleGearRatio = 8.14;
    SwerveConfig.SteeringkP = .6;
    SwerveConfig.ThrottlekP = .5;
    DriveTrain.addTab();
  }

  public void ControllerVibration() {
    RobotContainer.driverController.setRumble(mfeedback.CollisionDetected());
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
