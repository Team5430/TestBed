package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team5430.util.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TestBed extends SubsystemBase{
    
    public TestBed(){}

//constants
   
    SwerveModule moduleA = new SwerveModule(0, 1);
    SwerveModule moduleB = new SwerveModule(6,7);

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    public void motorConfig(){

    }
        //setAngle will set the directional angle
    public void drive(double angle, double power){
        moduleA.setAngle(deadzone(angle, power));
        moduleA.setThrottle(power);
        moduleB.setAngle(deadzone(angle, power));
        moduleB.setThrottle(power);
    }

    
   //**the wheel will go to the position that is greater than 0.2, otherwise stop power when less than or equal to*/
    public double deadzone(double angle, double power){
        double lastAngle;
//If the input given is less than 0.2 the rotation will reset to 0
    if(power >= -15){
        lastAngle = angle;
        if(power <= -15) {
        return lastAngle;
        } 
    }
        return angle;
  }

    public void angle(double angle, double distance ){
       moduleA.Angle(angle, distance); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(moduleA);
        SmartDashboard.putData(moduleB);
        SmartDashboard.putNumber("degrees", RobotContainer.driverJoystick.getDirectionDegrees());
    }

    
}
