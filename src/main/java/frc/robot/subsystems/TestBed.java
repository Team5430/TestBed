package frc.robot.subsystems;

import com.team5430.util.SwerveModuleConstants;
import com.kauailabs.navx.frc.AHRS;
import com.team5430.util.SwerveModuleGroup;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TestBed extends SubsystemBase {

  public TestBed() {}

  // constants

  public AHRS gyro = new AHRS(Port.kMXP);

  private SwerveModuleConstants mConfig = new SwerveModuleConstants();

  
  //swerve CANids; 0 through 7, even being drive motor, and odd being angle motor, following alphabetically,
  //Module A -> Module B -> Module C
  SwerveModuleGroup DriveTrain =
      new SwerveModuleGroup(4 , mConfig);


  public double lastAngle = 0;

  public void publishData() {
    SmartDashboard.putData("DriveTrain", DriveTrain);
    SmartDashboard.putData("Gyroscope", gyro);
    SmartDashboard.putNumber("degrees", RobotContainer.driverJoystick.getLeftStickDirectionDegrees());
    SmartDashboard.putNumber("magnitude", RobotContainer.driverJoystick.getLeftMagnitude());
  }


  // setAngle will set the directional angle
  public void drive(double x, double y, double rotation,  double breaking) {
    DriveTrain.Drive(new ChassisSpeeds(
                      MathUtil.applyDeadband(x, .3) * breaking,
                      MathUtil.applyDeadband(y, .3) * breaking,
                      rotation));
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


  @Override
  public void periodic() {

    SmartDashboard.updateValues();
  }
}