package com.team5430.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.function.DoubleSupplier;

public class SwerveModule implements Sendable {

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder magEncoder;
  private final String moduleName;

  private double angleRatio = 21.42857;
  private double driveRatio = 8.14;
  private double angle_kP = 0.65;
  private double drive_kP = .15;
  private double appliedPower;

  public SwerveModule(int AngleMotorCANid, int DriveMotorCANid, String Name) {
    angleMotor = new TalonFX(AngleMotorCANid);
    driveMotor = new TalonFX(DriveMotorCANid);
    magEncoder = new CANcoder(DriveMotorCANid + 1);

    moduleName = Name;

    motorConfig();
    SendableRegistry.addLW(this, moduleName);
  }

  private void motorConfig() {
    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    // goes towards closest value that is equivalent to wanted
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // Cancoder config
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    angleConfig.Feedback.SensorToMechanismRatio = 1;
    angleConfig.Feedback.FeedbackRemoteSensorID = magEncoder.getDeviceID();
    // used when there is not abs encoder
    // angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
    // proportional gains
    angleConfig.Slot0.kP = angle_kP;
    driveConfig.Slot0.kP = drive_kP;
    driveConfig.Feedback.SensorToMechanismRatio = driveRatio;

    // apply configurations
    angleMotor.getConfigurator().apply(angleConfig);
    driveMotor.getConfigurator().apply(driveConfig);

    // zero encoders
    angleMotor.setPosition(0);
    driveMotor.setPosition(0);
  }

  // any degree value will be turned into rotational value
  public void setAngle(double input) {
    angleMotor.setControl(new PositionDutyCycle(input / 360));
  }

  // TODO Switch to velocity; translate anything based on power as well
  // percentage targeted to power motor
  public void setThrottle(double throttle) {
    driveMotor.setControl(new DutyCycleOut(throttle));
    appliedPower = throttle;
  }

  // getters and setters
  public double getThrottle() {
    return appliedPower;
  }

  public void setAngleRatio(double ratio) {
    angleRatio = ratio;
    motorConfig();
  }

  public void setDriveRatio(double ratio) {
    driveRatio = ratio;
  }

  public void SetGains(double AngleMotorkP, double DriveMotorkP) {
    angle_kP = AngleMotorkP;
    drive_kP = DriveMotorkP;
  }

  public void StopAll() {
    setAngle(0);
    setThrottle(0);
  }

  public DoubleSupplier angleMotorEncoder = () -> angleMotor.getRotorPosition().getValue();

  public DoubleSupplier driveMotorEncoder = () -> driveMotor.getRotorPosition().getValue();

  public DoubleSupplier AbsoluteReading = () -> magEncoder.getAbsolutePosition().getValue();

  public DoubleSupplier PositionReading = () -> magEncoder.getPosition().getValue();

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

  public String getName() {
    return moduleName;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.setSafeState(this::StopAll);
    builder.addDoubleProperty("Angle Encoder", angleMotorEncoder, null);
    builder.addDoubleProperty("Drive Encoder", driveMotorEncoder, null);
    builder.addDoubleProperty("Absolute Encoder Reading", AbsoluteReading, null);
    builder.addDoubleProperty("Position Encoder Reading", PositionReading, null);
    builder.addDoubleProperty("Angle Motor kP", this::getAnglekP, this::setAnglekP);
    builder.addDoubleProperty("Drive Motor kP", this::getDrivekP, this::setDrivekP);
    builder.addDoubleProperty("Drive Power", this::getThrottle, null);
  }
}
