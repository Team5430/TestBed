package com.team5430.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private double angle = 360;
    private double angleRatio;
    private double driveRatio;

    private double angle_kP;
    private double drive_kP;


    public SwerveModule(int AngleMotorCANid, int DriveMotorCANid){
        angleMotor = new TalonFX(AngleMotorCANid);
        driveMotor = new TalonFX(DriveMotorCANid);
        motorConfig();
    }

    private void motorConfig(){

        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
        angleConfig.Slot0.kP = angle_kP;

        driveConfig.Slot0.kP = drive_kP;
        driveConfig.Feedback.SensorToMechanismRatio = driveRatio;


        angleMotor.getConfigurator().apply(angleConfig);
        driveMotor.getConfigurator().apply(driveConfig);

    }

    public void setAngle(double wanted){
        angleMotor.setControl(new PositionDutyCycle(wanted/angle));
    }

    public void setThrottle(double throttle){
        driveMotor.setControl(new DutyCycleOut(throttle));
    }

    public void setAngleRatio(double ratio){
        angleRatio = ratio;
    }

    public void setDriveRatio(double ratio){
        driveRatio = ratio;
    }

    public void SetGains(double AngleMotorkP, double DriveMotorkP){
        angle_kP = AngleMotorkP;
        drive_kP = DriveMotorkP;
    }


}
