package com.team5430.swerve;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public interface ModuleIO {
    
//this is what every swerve module should be able to do, real or not

    public enum CHARAZTERIZE{
        STEER,
        THROTTLE
    }

    public Rotation2d getRotation2d();

    public SwerveModuleState getState(boolean refresh);

    public void setState(SwerveModuleState state);

    public SwerveModulePosition getPosition(boolean refresh);

    public SwerveModulePosition getModuleDelta();
    
    public void setVoltage(Measure<Voltage> volts);

    public void Stop();

    

}
