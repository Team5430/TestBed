
package com.team5430.util;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.function.DoubleSupplier;

public class SwerveModule implements Sendable {

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder CANCoder; 
  private double Angle = 360;
  private double angleRatio = 1;
  private double driveRatio = 8.14;
  public double currentHeading;
  public double currentThrottle;

  private double angle_kP = 0.95;
  private double drive_kP = .15;
  private double magEncoderOffset;

  public SwerveModule(int AngleMotorCANid, int DriveMotorCANid, int CANCoderCANid, double offset) {
    angleMotor = new TalonFX(AngleMotorCANid);
    driveMotor = new TalonFX(DriveMotorCANid);
     CANCoder = new CANcoder(CANCoderCANid);

     magEncoderOffset = offset;
    motorConfig();
    SendableRegistry.addChild(this, angleMotor);
    SendableRegistry.addChild(this, driveMotor);
    SendableRegistry.addLW(this, "Swerve Module");
  }

  private void motorConfig() {
    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    
    // goes towards closest value that is equivalent to setpoint
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
    // proportional gains
    angleConfig.Slot0.kP = angle_kP;
    driveConfig.Slot0.kP = drive_kP;
    
    driveConfig.Feedback.SensorToMechanismRatio = driveRatio;

    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = magEncoderOffset;

    angleConfig.Feedback.FeedbackRemoteSensorID = CANCoder.getDeviceID();
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;



    // apply configurations
    angleMotor.getConfigurator().apply(angleConfig);
    driveMotor.getConfigurator().apply(driveConfig);
    CANCoder.getConfigurator().apply(encoderConfig);

    // zero encoders

    angleMotor.setPosition(magEncoderOffset);
   }

  


  // any degree value will be turned into rotational value
  public void setAngle(double input) { 

  angleMotor.setControl(new PositionDutyCycle(input / Angle ));
    currentHeading = input/Angle;

  }

  // power to motor that moves the wheel
  public void setThrottle(double throttle) {
    driveMotor.setControl(new DutyCycleOut(throttle));
    currentThrottle = throttle;
  }

  // getters and setters
  public double getThrottle() {
    return currentThrottle;
  }

  public double getHeading(){
    return currentHeading;
  }

 public SwerveModuleState getCurrentState(){
  return new SwerveModuleState(currentThrottle, Rotation2d.fromDegrees(currentHeading));
 }

  public void StopAll() {
    setAngle(0);
    setThrottle(0);
  }

  public DoubleSupplier angleMotorEncoder = () -> angleMotor.getRotorPosition().getValue();

  public DoubleSupplier driveMotorEncoder = () -> driveMotor.getRotorPosition().getValue();

  public DoubleSupplier CANcoderEncoder = () -> CANCoder.getAbsolutePosition().getValue();

  public void SetAngleEncoder(double set) {
    setAngle(set);
  }

  public double getDriveMotorEncoder() {
    return driveMotor.getRotorPosition().getValueAsDouble();
  }

  public void SetDriveEncoder(double input) {
    driveMotor.setControl(new PositionDutyCycle(input));
  }

  public double getAnglekP() {
    return angle_kP;
  }

  public void setAnglekP(double kP) {
    angle_kP = kP;
  }

  public double getDrivekP() {
    return drive_kP;
  }

  public void setDrivekP(double kP) {
    drive_kP = kP;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module Telemetry");
    builder.setActuator(true);
    builder.setSafeState(this::StopAll);
    builder.addDoubleProperty("Angle Encoder", angleMotorEncoder, null);
    builder.addDoubleProperty("CANCoder", CANcoderEncoder, null);
    builder.addDoubleProperty("Drive Encoder", driveMotorEncoder, null);
    builder.addDoubleProperty("Angle Motor kP", this::getAnglekP, null);
    builder.addDoubleProperty("Drive Motor kP", this::getDrivekP, null);
    builder.addDoubleProperty("Drive Motor Power", this::getThrottle, null);
  }

  
}
