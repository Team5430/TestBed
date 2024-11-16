package com.team5430.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {

  protected TalonFX angleMotor;
  protected TalonFX driveMotor;
  protected CANcoder CANCoder;

  // constants
  protected double angleRatio = 1;
  protected double driveRatio = 8.14;

  protected SwerveModulePosition internalPosition = new SwerveModulePosition();
  protected SwerveModuleState internalState = new SwerveModuleState();

  private StatusSignal<Double> drivePosition;
  private StatusSignal<Double> driveVelocity;
  private StatusSignal<Double> anglePosition;
  private StatusSignal<Double> angleVelocity;
  protected BaseStatusSignal[] signals;

  protected double angle_kP = 0.95;
  protected double drive_kP = .15;
  private final double magEncoderOffset;

  public SwerveModule(int AngleMotorCANid, int DriveMotorCANid, int CANCoderCANid, double offset) {
    angleMotor = new TalonFX(AngleMotorCANid);
    driveMotor = new TalonFX(DriveMotorCANid);
    CANCoder = new CANcoder(CANCoderCANid);

    magEncoderOffset = offset;
    motorConfig();

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    anglePosition = angleMotor.getPosition();
    angleVelocity = angleMotor.getVelocity();

    // data as statusSignals
    signals = new BaseStatusSignal[4];
    signals[0] = drivePosition;
    signals[1] = driveVelocity;
    signals[2] = anglePosition;
    signals[3] = angleVelocity;
  }

  private void motorConfig() {

    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
    // proportional gains
    angleConfig.Slot0.kP = angle_kP;
    driveConfig.Slot0.kP = drive_kP;

    // voltage config

    // max amperage
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
    driveConfig.Feedback.SensorToMechanismRatio = driveRatio;
    // max of 10 volts allows
    driveConfig.Voltage.PeakForwardVoltage = 10;
    driveConfig.Voltage.PeakReverseVoltage = -10;

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

  public void setState(SwerveModuleState state) {
    var optimize = SwerveModuleState.optimize(state, getState(true).angle);
    // Heading
    double wantedRad = optimize.angle.getRadians();
    // flip wanted to the closest direction if needed
    // SmartDashboard.putNumber("Angle", wantedRad);

    angleMotor.setControl(new PositionDutyCycle(wantedRad / (2 * Math.PI)));

    // Throttle; cosine compensation
    var currentAngle = state.angle;
    state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
    // get wanted
    double wantedVelocity = state.speedMetersPerSecond;
    driveMotor.setControl(new VelocityDutyCycle(wantedVelocity));
  }

  /**
   * SwerveModulePosition is an object which contains the modules position and modules angle
   *
   * @return The current position of the module
   */
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      drivePosition.refresh();
      driveVelocity.refresh();
      anglePosition.refresh();
      angleVelocity.refresh();
    }

    double driveRotations =
        BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angleRotations =
        BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

    internalPosition.distanceMeters = driveRotations;
    internalPosition.angle = Rotation2d.fromRotations(angleRotations);

    return internalPosition;
  }

  public SwerveModuleState getState(boolean refresh) {

    if (refresh) {
      driveVelocity.refresh();
      anglePosition.refresh();
    }

    internalState.angle = Rotation2d.fromDegrees(anglePosition.getValue());
    internalState.speedMetersPerSecond = driveVelocity.getValue();

    return internalState;
  }

  public void invertThrottle(boolean input) {

    driveMotor.setInverted(input);
  }

  public void Stop() {
    angleMotor.stopMotor();
    driveMotor.stopMotor();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module Telemetry");
    builder.setActuator(true);
    builder.setSafeState(this::Stop);
    builder.addDoubleProperty("Drive Motor Power", null, null);
  }
}
