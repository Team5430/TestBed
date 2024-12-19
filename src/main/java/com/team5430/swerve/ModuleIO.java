package com.team5430.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    
//this is what every swerve module should be able to do, real or not
    public Rotation2d getRotation2d();

    public SwerveModuleState getState(boolean refresh);

    public void setState(SwerveModuleState state);

    public SwerveModulePosition getPosition(boolean refresh);

    public SwerveModulePosition getModuleDelta();
    
    public void Stop();

    

}
