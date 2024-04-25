package com.team5430.util;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SwerveModule implements Sendable {

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private double Angle = 360;
    private double angleRatio = 21.42857;
    private double driveRatio = 8.14;

    private double angle_kP = 0.6;
    private double drive_kP = .15;
    private double appliedPower;

    public SwerveModule(int AngleMotorCANid, int DriveMotorCANid){
        angleMotor = new TalonFX(AngleMotorCANid);
        driveMotor = new TalonFX(DriveMotorCANid);
        motorConfig();
        SendableRegistry.addChild(this, angleMotor);
        SendableRegistry.addChild(this, driveMotor);
        SendableRegistry.addLW(this, "Swerve Module");
    }

    private void motorConfig(){
//create config objects
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

//goes towards closest value that is equivalent to setpoint
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
//gear ratio
        angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
//proportional gains
        angleConfig.Slot0.kP = angle_kP;
        driveConfig.Slot0.kP = drive_kP;
        driveConfig.Feedback.SensorToMechanismRatio = driveRatio;

//apply configurations
        angleMotor.getConfigurator().apply(angleConfig);
        driveMotor.getConfigurator().apply(driveConfig);

//zero encoders
        angleMotor.setPosition(0);
        driveMotor.setPosition(0);

    }

//any degree value will be turned into rotational value
    public void setAngle(double input){
        angleMotor.setControl(new PositionDutyCycle(input/Angle));
    }

//power to motor that moves the wheel
    public void setThrottle(double throttle){
        driveMotor.setControl(new DutyCycleOut(throttle));
        appliedPower = throttle;
    }
    
//getters and setters
    public double getThrottle(){
        return appliedPower;
    }

    public void setAngleRatio(double ratio){
        angleRatio = ratio;
        motorConfig();
    }

    public void setDriveRatio(double ratio){
        driveRatio = ratio;
        motorConfig();
    }

    public void setDrivekP(double kP){
         drive_kP = kP;
         motorConfig();
    }

    public double getDrivekP(){
        return drive_kP;
    }

    public void setAnglekP(double kP){
        angle_kP = kP;
        motorConfig();
    }

    public double getAnglekP(){
        return angle_kP;
    }

    public void SetDriveEncoder(double value){
        driveMotor.setPosition(value);
    }

    public void SetAngleEncoder(double value){
        angleMotor.setPosition(value);
    }

    //safe function to fallback to
    public void StopAll(){
        setAngle(0);
        setThrottle(0);
    }

    public DoubleSupplier driveMotorEncoder(){
       return () -> driveMotor.getRotorPosition().getValueAsDouble();
    }

    public DoubleSupplier angleMotorEncoder(){
        return () -> angleMotor.getRotorPosition().getValueAsDouble();
    }

//Calculations to find the rotations needed to travel a given distance
    public void Angle(double angle, double distance){
        setAngle(angle);
        double circumfrence = 3.75 * Math.PI;
        double rotations = distance/circumfrence;
        driveMotor.setPosition(0);   
        driveMotor.setControl(new PositionDutyCycle(rotations));

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module Telemetry");
        builder.setActuator(true);
        builder.setSafeState(this::StopAll);
        builder.addDoubleProperty("Angle Encoder", angleMotorEncoder(), this::SetAngleEncoder);
        builder.addDoubleProperty("Drive Encoder", driveMotorEncoder(), this::SetDriveEncoder);
        builder.addDoubleProperty("Angle Motor kP", this::getAnglekP, this::setAnglekP);
        builder.addDoubleProperty("Drive Motor kP",this::getDrivekP, this::setDrivekP);
        builder.addDoubleProperty("Drive Motor Power", this::getThrottle , this::setThrottle);

    }


}
