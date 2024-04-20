package frc.robot.subsystems;

import com.team5430.util.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TestBed extends SubsystemBase{
    
    public TestBed(){}

//constants
    private double kP = .3;

    SwerveModule module = new SwerveModule(0, 1);

    public void motorConfig(){
        module.SetGains(kP, kP);
    }
        //setAngle will set the directional angle
    public void drive(double angle, double power){
        module.setAngle(deadzone(angle, power));
        module.setThrottle(power);
    }

    public double deadzone(double angle, double power){
        //If the input given is less than 0.2 the rotation will reset to 0
        if(power >= -.2) {
            return 0;
        } else {    
        return angle;
        //if the input given is greater than 0.2 it will to the joystick degree position
    }
    }

    public void angle(double angle, double distance ){
       module.Angle(angle, distance); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive Motor", module.driveMotorEncoder());
        SmartDashboard.putNumber("Angle Motor", module.angleMotorEncoder());
        SmartDashboard.putNumber("degrees", RobotContainer.driverJoystick.getDirectionDegrees());
    }

    
}
