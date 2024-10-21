package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team5430.util.SwerveModuleConstants;
import com.team5430.util.SwerveModuleGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestBed extends SubsystemBase {

  public TestBed() {}

  // constants

  public AHRS mGyro = new AHRS(Port.kMXP);

  private SwerveModuleConstants mConfig = new SwerveModuleConstants();

  
  //swerve CANids; 0 through 7, even being drive motor, and odd being angle motor, following alphabetically,
  //Module A -> Module B -> Module C
 private SwerveModuleGroup DriveTrain =
      new SwerveModuleGroup(4, mConfig);




  public void publishData() {
    SmartDashboard.putData("Gyroscope", mGyro);

  }
  

  // setAngle will set the directional angle
  public void Drive(ChassisSpeeds input, Rotation2d robotAngle , boolean isFieldCentric) {

  if(isFieldCentric){

    DriveTrain.FieldCentricDrive(input, robotAngle);
      }else{
    DriveTrain.Drive(input);
    }

  }

  

  public Command DriveToDistance(double Distance){
    return new InstantCommand(() -> DriveTrain.DriveToDistance(Distance), this);
  }

    public double lastAngle = 0;
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

    DriveTrain.publishData();

    SmartDashboard.updateValues();
  }
}