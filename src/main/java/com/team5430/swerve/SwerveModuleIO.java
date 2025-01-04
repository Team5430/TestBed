package com.team5430.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

/**
 * The {@code SwerveModule} class represents a single swerve throttle module,
 * including its motors and encoder, and provides methods to control and
 * retrieve the module's state.
 * <p>
 * This class handles the initialization, configuration, and control of
 * the swerve module's throttle and steering motors and the CANcoder encoder.
 * The swerve module's state (including position and velocity) can be set
 * and retrieved using this class.
 */
public class SwerveModuleIO implements ModuleIO {

  protected TalonFX steeringMotor;
  protected TalonFX throttleMotor;
  protected CANcoder CANCoder;

  protected int ModuleNumber;

  protected SwerveModulePosition internalPosition = new SwerveModulePosition();
  protected SwerveModuleState internalState = new SwerveModuleState();
  protected SwerveModuleConstants constants = new SwerveModuleConstants();

  private final StatusSignal<Double> throttlePosition;
  private final StatusSignal<Double> throttleVelocity;
  private final StatusSignal<Double> steeringPosition;
  private final StatusSignal<Double> angularVelocity;

  protected MotionMagicExpoVoltage m_angleSetter = new MotionMagicExpoVoltage(0); 
  protected BaseStatusSignal[] signals;

  
  protected CHARAZTERIZE type = CHARAZTERIZE.STEER;
  /**
   * Constructs a new {@code SwerveModule}.
   *
   * @param moduleNumber The module number used to index into configuration arrays.
   * @param config The configuration constants for the swerve module.
   */
  public SwerveModuleIO(int moduleNumber, SwerveModuleConstants config) {
    this.constants = config;
    this.ModuleNumber = moduleNumber;

    // Initialize motors and encoder with their respective CAN IDs from the configuration
    this.steeringMotor = new TalonFX(config.STEERING_MODULE_MOTORID[moduleNumber]);
    this.throttleMotor = new TalonFX(config.THROTTLE_MODULE_MOTORID[moduleNumber]);
    this.CANCoder = new CANcoder(config.CANCODER_ID[moduleNumber]);

    //configure the motors 
    motorConfig();

    // Initialize sensor signals for position and velocity
    this.throttlePosition = throttleMotor.getPosition();
      throttlePosition.setUpdateFrequency(25);
    this.throttleVelocity = throttleMotor.getVelocity();
      throttleVelocity.setUpdateFrequency(25);
    this.steeringPosition = steeringMotor.getPosition();
      steeringPosition.setUpdateFrequency(25);
    this.angularVelocity = steeringMotor.getVelocity();
      angularVelocity.setUpdateFrequency(10);

    // Store signals in an array for easier management
    this.signals = new BaseStatusSignal[4];
    this.signals[0] = throttlePosition;
    this.signals[1] = throttleVelocity;
    this.signals[2] = steeringPosition;
    this.signals[3] = angularVelocity;
  }

  public SwerveModuleIO() {
    this.throttlePosition = null;
    this.throttleVelocity = null;
    this.steeringPosition = null;
    this.angularVelocity = null;
  }

  private void motorConfig(){
        // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = 1;
    // gains
    var angleSlot0 = angleConfig.Slot0;
      angleSlot0.kS = constants.STEER_KS;
      angleSlot0.kV = constants.STEER_KV;
      angleSlot0.kA = constants.STEER_KA;
      angleSlot0.kP = constants.STEER_KP;
      angleSlot0.kI = constants.STEER_KI;
      angleSlot0.kD = constants.STEER_KD;

    driveConfig.Slot0.kP = constants.THROTTLE_KP;
    // max amperage
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
    driveConfig.Feedback.SensorToMechanismRatio = constants.THROTTLE_RATIO;
    // max of 10 volts allows
    driveConfig.Voltage.PeakForwardVoltage = 10;
    driveConfig.Voltage.PeakReverseVoltage = -10;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = constants.STEERING_MODULE_OFFSET[ModuleNumber];
    angleConfig.Feedback.FeedbackRemoteSensorID = CANCoder.getDeviceID();
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    
    //motion magic
    var angleMotionMagic = angleConfig.MotionMagic;
      angleMotionMagic.MotionMagicCruiseVelocity = (100 / constants.STEER_RATIO);
      angleMotionMagic.MotionMagicAcceleration 
        =  angleConfig.MotionMagic.MotionMagicCruiseVelocity/ .1;
      angleMotionMagic.MotionMagicExpo_kV = (.12 * constants.STEER_RATIO);
      angleMotionMagic.MotionMagicExpo_kA = (.1);
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

    
    // apply configurations
    steeringMotor.getConfigurator().apply(angleConfig);
    throttleMotor.getConfigurator().apply(driveConfig);
    CANCoder.getConfigurator().apply(encoderConfig);
    // zero encoders
    steeringMotor.setPosition(constants.STEERING_MODULE_OFFSET[ModuleNumber]);
  }
  
  /**
   * Sets the state of the swerve module (angle   and speed).
   *
   * @param state The desired state (angle and speed) for the module.
   */
  @Override
  public void setState(SwerveModuleState state) {
    // Optimize the state angle to avoid rotating more than 180 degrees
    var optimize = SwerveModuleState.optimize(state, getState(true).angle);
    double wantedRotations = optimize.angle.getRotations();

    // Set the steering motor to the desired angle
    steeringMotor.setControl(new PositionDutyCycle(wantedRotations));

    // Adjust the speed based on the current and desired angles
    var currentAngle = state.angle;
    state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
    double wantedVelocity = state.speedMetersPerSecond;

    // Set the throttle motor to the desired speed
    throttleMotor.setControl(new VelocityDutyCycle(wantedVelocity));
  }

  /**
   * SwerveModulePosition is an object which contains the module's position and angle.
   *
   * @param refresh If true, updates the position and angle values by refreshing sensor readings.
   * @return The current position of the module.
   */
  @Override
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      // Refresh sensor readings
      throttlePosition.refresh();
      throttleVelocity.refresh();
      steeringPosition.refresh();
      angularVelocity.refresh();
    }

    // Get compensated throttle rotations and angle rotations
    double throttleRotations = BaseStatusSignal.getLatencyCompensatedValue(throttlePosition, throttleVelocity);
    double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(steeringPosition, angularVelocity);

    // Set the internal position with updated values
    internalPosition.distanceMeters = throttleRotations;
    internalPosition.angle = Rotation2d.fromRotations(angleRotations);

    return internalPosition;
  }


  /**
   * Retrieves the current state (angle and speed) of the swerve module.
   *
   * @param refresh If true, updates the state values by refreshing sensor readings.
   * @return The current state of the swerve module.
   */
  @Override
  public SwerveModuleState getState(boolean refresh) {
    if (refresh) {
      // Refresh readings
      throttleVelocity.refresh();
      steeringPosition.refresh();
    }

    // Update internal state with current sensor values
    internalState.angle = Rotation2d.fromDegrees(steeringPosition.getValue());
    internalState.speedMetersPerSecond = throttleVelocity.getValue();

    return internalState;
  }

  /**
   * Sets the voltage output of the swerve module's motor.
   *
   * @param voltage The desired voltage output.
   * @param type The type of motor to set the voltage to (steering or throttle).
   */
  @Override
  public void setVoltage(Measure<Voltage> volts){
    
    //determine which motor to set voltage to
    switch (type) {
      
      case STEER:
         steeringMotor.setControl(new VoltageOut(volts.magnitude()));  
        break;
    
      case THROTTLE:
        throttleMotor.setControl(new VoltageOut(volts.magnitude()));
        break;
    }

  }

  public void setChracterize(CHARAZTERIZE type){
    this.type = type;
  }
   /**
   * Retrieves the change in the module's position since the last update.
   */
  @Override
  public SwerveModulePosition getModuleDelta(){
    return new SwerveModulePosition(getPosition(false).distanceMeters - internalPosition.distanceMeters, internalPosition.angle);
  }
 
  
  /**
   * Retrieves the current angle of the swerve module.
   *
   * @return The current angle of the swerve module.
   */
  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(steeringPosition.getValue());
  }

 
  /**
   * Stops both the steering and throttle motors of the swerve module.
   */
  @Override
  public void Stop() {
    // Stop the motors
    steeringMotor.stopMotor();
    throttleMotor.stopMotor();
  }
}