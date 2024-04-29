package com.team5430.util;

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

  public SwerveModule(int AngleMotorCANid, int DriveMotorCANid) {
    angleMotor = new TalonFX(AngleMotorCANid);
    driveMotor = new TalonFX(DriveMotorCANid);
    motorConfig();
    SendableRegistry.addChild(this, angleMotor);
    SendableRegistry.addChild(this, driveMotor);
    SendableRegistry.addLW(this, "Swerve Module");
  }

  private void motorConfig() {
    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    // goes towards closest value that is equivalent to setpoint
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
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

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
  }
}
