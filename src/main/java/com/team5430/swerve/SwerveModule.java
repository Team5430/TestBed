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
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The {@code SwerveModule} class represents a single swerve drive module,
 * including its motors and encoder, and provides methods to control and
 * retrieve the module's state.
 * <p>
 * This class handles the initialization, configuration, and control of
 * the swerve module's drive and steering motors and the CANcoder encoder.
 * The swerve module's state (including position and velocity) can be set
 * and retrieved using this class.
 */
public class SwerveModule {

  protected TalonFX steeringMotor;
  protected TalonFX throttleMotor;
  protected CANcoder CANCoder;

  protected int ModuleNumber;

  protected SwerveModulePosition internalPosition = new SwerveModulePosition();
  protected SwerveModuleState internalState = new SwerveModuleState();
  protected SwerveModuleConstants constants = new SwerveModuleConstants();

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> steeringPosition;
  private final StatusSignal<Double> angularVelocity;

  protected BaseStatusSignal[] signals;

  /**
   * Constructs a new {@code SwerveModule}.
   *
   * @param moduleNumber The module number used to index into configuration arrays.
   * @param config The configuration constants for the swerve module.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants config) {
    this.constants = config;
    this.ModuleNumber = moduleNumber;

    // Initialize motors and encoder with their respective CAN IDs from the configuration
    this.steeringMotor = new TalonFX(constants.STEERING_MODULE_MOTORID[moduleNumber]);
    this.throttleMotor = new TalonFX(constants.THROTTLE_MODULE_MOTORID[moduleNumber]);
    this.CANCoder = new CANcoder(constants.CANCODER_ID[moduleNumber]);

    // Apply the motor and encoder configurations
    motorConfig();

    // Initialize sensor signals for position and velocity
    this.drivePosition = throttleMotor.getPosition();
      drivePosition.setUpdateFrequency(25);
    this.driveVelocity = throttleMotor.getVelocity();
      driveVelocity.setUpdateFrequency(25);
    this.steeringPosition = steeringMotor.getPosition();
      steeringPosition.setUpdateFrequency(25);
    this.angularVelocity = steeringMotor.getVelocity();
      angularVelocity.setUpdateFrequency(10);

    // Store signals in an array for easier management
    this.signals = new BaseStatusSignal[4];
    this.signals[0] = drivePosition;
    this.signals[1] = driveVelocity;
    this.signals[2] = steeringPosition;
    this.signals[3] = angularVelocity;
  }

  public SwerveModule() {
    this.drivePosition = null;
    this.driveVelocity = null;
    this.steeringPosition = null;
    this.angularVelocity = null;
  }
  /**
   * Configures the motors and encoder using the provided configuration constants.
   */
  private void motorConfig() {
    try {
      // Apply configurations to the steering motor, drive motor, and encoder
      
    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = constants.steerRatio;
    // proportional gains
    angleConfig.Slot0.kP = constants.steer_kP;
    driveConfig.Slot0.kP = constants.throttle_kP;

    // max amperage
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
    driveConfig.Feedback.SensorToMechanismRatio = constants.throttleRatio;
    // max of 10 volts allows
    driveConfig.Voltage.PeakForwardVoltage = 10;
    driveConfig.Voltage.PeakReverseVoltage = -10;

    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = constants.STEERING_MODULE_OFFSET[ModuleNumber];

    angleConfig.Feedback.FeedbackRemoteSensorID = CANCoder.getDeviceID();
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // apply configurations
    steeringMotor.getConfigurator().apply(angleConfig);
    throttleMotor.getConfigurator().apply(driveConfig);
    CANCoder.getConfigurator().apply(encoderConfig);

    // zero encoders

    steeringMotor.setPosition(constants.STEERING_MODULE_OFFSET[ModuleNumber]);


    } catch (Exception e) {
      // Report any errors encountered during configuration
      DriverStation.reportError("Error setting Swerve Module: " + ModuleNumber + " configuration", e.getStackTrace());
    }
  }

  /**
   * Sets the state of the swerve module (angle and speed).
   *
   * @param state The desired state (angle and speed) for the module.
   */
  public void setState(SwerveModuleState state) {
    // Optimize the state angle to avoid rotating more than 180 degrees
    var optimize = SwerveModuleState.optimize(state, getState(true).angle);
    double wantedRad = optimize.angle.getRadians();

    // Set the steering motor to the desired angle
    steeringMotor.setControl(new PositionDutyCycle(wantedRad / (2 * Math.PI)));

    // Adjust the speed based on the current and desired angles
    var currentAngle = state.angle;
    state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
    double wantedVelocity = state.speedMetersPerSecond;

    // Set the drive motor to the desired speed
    throttleMotor.setControl(new VelocityDutyCycle(wantedVelocity));
  }

  /**
   * SwerveModulePosition is an object which contains the module's position and angle.
   *
   * @param refresh If true, updates the position and angle values by refreshing sensor readings.
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      // Refresh sensor readings
      drivePosition.refresh();
      driveVelocity.refresh();
      steeringPosition.refresh();
      angularVelocity.refresh();
    }

    // Get compensated drive rotations and angle rotations
    double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(steeringPosition, angularVelocity);

    // Set the internal position with updated values
    internalPosition.distanceMeters = driveRotations;
    internalPosition.angle = Rotation2d.fromRotations(angleRotations);

    return internalPosition;
  }

  /**
   * Retrieves the current state (angle and speed) of the swerve module.
   *
   * @param refresh If true, updates the state values by refreshing sensor readings.
   * @return The current state of the swerve module.
   */
  public SwerveModuleState getState(boolean refresh) {
    if (refresh) {
      // Refresh readings
      driveVelocity.refresh();
      steeringPosition.refresh();
    }

    // Update internal state with current sensor values
    internalState.angle = Rotation2d.fromDegrees(steeringPosition.getValue());
    internalState.speedMetersPerSecond = driveVelocity.getValue();

    return internalState;
  }

 
  /**
   * Stops both the steering and drive motors of the swerve module.
   */
  public void Stop() {
    // Stop the motors
    steeringMotor.stopMotor();
    throttleMotor.stopMotor();
  }
}