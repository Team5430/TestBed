package frc.robot.subsystems;

import com.team5430.util.SwerveModule;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestBed extends SubsystemBase{
    
    public TestBed(){}

//constants
    private double kP = .15;
    private double driveGearRatio;
    private double angleGearRatio;

    SwerveModule module = new SwerveModule(0, 1);

    public void motorConfig(){
        module.SetGains(kP, kP);
        module.setAngleRatio(angleGearRatio);
        module.setDriveRatio(driveGearRatio);
    }

    public void drive(double direction, double power){
        module.setAngle(direction);
        module.setThrottle(power);
    }
}
