package com.team5430.util;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team5430.util.configs.SwerveConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SwerveModule implements Sendable {

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private double Angle = 360;
  private double angle_kP = 0.15;
  private double drive_kP = .15;
  private double appliedPower;

  public SwerveModule(int AngleMotorCANid, int DriveMotorCANid, SwerveConfig config) {
    angleMotor = new TalonFX(AngleMotorCANid);
    driveMotor = new TalonFX(DriveMotorCANid);
    motorConfig(config);
    SendableRegistry.addChild(this, angleMotor);
    SendableRegistry.addChild(this, driveMotor);
    SendableRegistry.addLW(this, "Swerve Module");
  }

  private void motorConfig(SwerveConfig config) {

    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    // goes towards closest value that is equivalent to setpoint
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = config.PhysicalProperties.AngularGearRatio;
    driveConfig.Feedback.SensorToMechanismRatio = config.PhysicalProperties.DriveGearRatio;

    // proportional gains
    angleConfig.Slot0.kP = config.Software.Angular_kP;
    driveConfig.Slot0.kP = config.Software.Drive_kP;

    // apply configurations
    angleMotor.getConfigurator().apply(angleConfig);
    driveMotor.getConfigurator().apply(driveConfig);

    // zero encoders
    angleMotor.setPosition(0);
    driveMotor.setPosition(0);
  }

  // any degree value will be turned into rotational value
  public void setAngle(double input) {
    angleMotor.setControl(new PositionDutyCycle(input / Angle));
  }

  // power to motor that moves the wheel
  public void setThrottle(double throttle) {
    driveMotor.setControl(new DutyCycleOut(throttle));
    appliedPower = throttle;
  }

  // getters and setters
  public double getThrottle() {
    return appliedPower;
  }

  public void StopAll(){
    setAngle(0);
    setThrottle(0);
  }

//getters and setters
  public double angleMotorEncoder(){
    return angleMotor.getRotorPosition().getValueAsDouble();
  }

 public void SetAngleEncoder(double set){
    setAngle(set);
 }

  public double driveMotorEncoder(){
    return driveMotor.getRotorPosition().getValueAsDouble();
  }

  public void SetDriveEncoder(double input){
    driveMotor.setControl(new PositionDutyCycle(input));
  }

  public double getAnglekP(){
      return angle_kP;
  }

  public void setAnglekP(double kP){
    angle_kP = kP;
  }

  public double getDrivekP(){
    return drive_kP;
  }

  public void setDrivekP(double kP){ 
    drive_kP = kP;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
   builder.setSmartDashboardType("Swerve Module Telemetry");
        builder.setActuator(true);
        builder.setSafeState(this::StopAll);
        builder.addDoubleProperty("Angle Encoder", this::angleMotorEncoder, this::SetAngleEncoder);
        builder.addDoubleProperty("Drive Encoder", this::driveMotorEncoder, this::SetDriveEncoder);
        builder.addDoubleProperty("Angle Motor kP", this::getAnglekP, this::setAnglekP);
        builder.addDoubleProperty("Drive Motor kP",this::getDrivekP, this::setDrivekP);
        builder.addDoubleProperty("Drive Motor Power", this::getThrottle , this::setThrottle);

  }
}
