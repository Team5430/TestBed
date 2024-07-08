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
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.DoubleSupplier;

public class SwerveModule implements Sendable {

  private TalonFX SteeringMotor;
  private TalonFX ThrottleMotor;
  private CANcoder magEncoder;

  private double appliedPower;
  private SwerveModuleConstants _config;
  private final String moduleName;

  /**
   * Constructor for individual Swerve Module of any kind, that is supported by TalonFX motors
   *
   * @param config configuration object to tend to your specific swerve module, to setup, use CANid
   * @see SwerveModuleConstants for usage details
   */
  public SwerveModule(SwerveModuleConstants config) {

    moduleName = config.Name;

    try {

      SteeringMotor = new TalonFX(config.SteeringMotorCANid);
      ThrottleMotor = new TalonFX(config.ThrottleMotorCANid);
      magEncoder = new CANcoder(config.CANCoderCANid);

      SendableRegistry.addLW(this, moduleName);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError(moduleName, e.getStackTrace());
    }

    motorConfig();
  }

  /**
   * If you insert values for CANid here, setting it in your config is not required
   *
   * @see SwerveModuleConstants
   */
  public SwerveModule(
          int SteeringMotorCANid,
          int ThrottleMotorCANid,
          int CANCoderCANid,
          SwerveModuleConstants config) {

    moduleName = "Module" + SteeringMotorCANid;

    try {

      SteeringMotor = new TalonFX(SteeringMotorCANid);
      ThrottleMotor = new TalonFX(ThrottleMotorCANid);
      magEncoder = new CANcoder(CANCoderCANid);

      SendableRegistry.addLW(this, moduleName);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError(moduleName, e.getStackTrace());
    }
    _config = config;
    motorConfig();
  }

  private void motorConfig() {
    // create config objects
    TalonFXConfiguration SteeringConfig = new TalonFXConfiguration();
    TalonFXConfiguration ThrottleConfig = new TalonFXConfiguration();
    // goes towards closest value that is equivalent to wanted
    SteeringConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // Cancoder config
    SteeringConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    SteeringConfig.Feedback.FeedbackRemoteSensorID = magEncoder.getDeviceID();
    // used when there is not abs encoder
    // SteeringConfig.Feedback.SensorToMechanismRatio = SteeringRatio;
    // proportional gains
    SteeringConfig.Slot0.kP = _config.SteeringkP;
    SteeringConfig.Feedback.SensorToMechanismRatio = _config.SteeringGearRatio;
    ThrottleConfig.Slot0.kP = _config.ThrottlekP;
    ThrottleConfig.Feedback.SensorToMechanismRatio = _config.ThrottleGearRatio;

    // apply configurations
    SteeringMotor.getConfigurator().apply(SteeringConfig);
    ThrottleMotor.getConfigurator().apply(ThrottleConfig);

    // zero encoders
    SteeringMotor.setPosition(0);
    ThrottleMotor.setPosition(0);
  }

  // any degree value will be turned into rotational value
  public void setSteering(double input) {
    SteeringMotor.setControl(new PositionDutyCycle(input / 360));
  }

  // TODO Switch to velocity; translate anything based on power as well
  // percentage targeted to power motor
  public void setThrottle(double throttle) {
    ThrottleMotor.setControl(new DutyCycleOut(throttle));
    appliedPower = throttle;
  }

  public DoubleSupplier SteeringMotorEncoder = () -> SteeringMotor.getRotorPosition().getValue();

  public DoubleSupplier ThrottleMotorEncoder = () -> ThrottleMotor.getRotorPosition().getValue();

  public DoubleSupplier AbsoluteReading = () -> magEncoder.getAbsolutePosition().getValue();

  public DoubleSupplier PositionReading = () -> magEncoder.getPosition().getValue();

  public String getName() {
    return moduleName;
  }

  public double getThrottle() {
    return appliedPower;
  }

  public void StopAll() {
    setSteering(0);
    setThrottle(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.setSafeState(this::StopAll);
    builder.addDoubleProperty("Steering Encoder", SteeringMotorEncoder, null);
    builder.addDoubleProperty("Throttle Encoder", ThrottleMotorEncoder, null);
    builder.addDoubleProperty("Absolute Encoder Reading", AbsoluteReading, null);
    builder.addDoubleProperty("Position Encoder Reading", PositionReading, null);
    builder.addDoubleProperty("Throttle Power", this::getThrottle, null);
  }
}
