package com.team5430.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

  protected BaseStatusSignal[] signals;

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
    this.steeringMotor = config.buildSteerMotor(moduleNumber);
    this.throttleMotor = config.buildThrottleMotor(moduleNumber);
    this.CANCoder = config.buildCancoder(moduleNumber);

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

  /**
   * Sets the state of the swerve module (angle   and speed).
   *
   * @param state The desired state (angle and speed) for the module.
   */
  @Override
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