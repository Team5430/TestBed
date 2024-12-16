package com.team5430.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveModuleConstants {

  // Maximum velocity in meters per second
  public double MAX_VELOCITY_MPS = 5;

  // Maximum angular velocity in radians per second
  public double MAX_OMEGA_RADIANS = 10;

  // Radius of the drive base in meters
  public double DRIVE_BASE_RADIUS = .567;

  // Steering and throttle ratios
  public double steerRatio = 150/7;
  public double throttleRatio = 8.14;

  // PID constants for steering and throttle
  protected double steer_kP = 0.95;
  protected double throttle_kP = .15;

  // Module-specific offsets for steering
  // All arrays follow the order of A = 0, B = 1, C = 2, D = 3
  public double[] STEERING_MODULE_OFFSET = {0.083, 0.02954, .27416, .44789};

  // CAN IDs for steering motors
  public int[] STEERING_MODULE_MOTORID = {0, 2, 4, 6};

  // CAN IDs for throttle motors
  public int[] THROTTLE_MODULE_MOTORID = {1, 3, 5, 7};

  // CAN IDs for encoders
  public int[] CANCODER_ID = {0, 1, 2, 3};

  // Module locations for kinematics calculations
  protected Translation2d[] ModuleLocations = {
          new Translation2d(0.267, 0.267), // Back left (A)
          new Translation2d(0.267, -0.267),   // Front right (B)
          new Translation2d(-0.267, 0.267),  // Front left (C)
          new Translation2d(-0.267, -0.267)   // Back right (D)
  };

  // Swerve drive kinematics
  public SwerveDriveKinematics Kinematics =
          new SwerveDriveKinematics(ModuleLocations);

  // Autonomous speed configurations PATHPLANNER ONLY COMMENT OUT IF NOT NEEDED
  public HolonomicPathFollowerConfig autoFollowerConfig =
          new HolonomicPathFollowerConfig(
                  new PIDConstants(5),            // Chassis PID constants
                  new PIDConstants(2),            // Theta PID constants
                  MAX_VELOCITY_MPS,               // Max velocity
                  DRIVE_BASE_RADIUS,              // Drive base radius
                  new ReplanningConfig());        // Replanning configurations

  public PPHolonomicDriveController pathFollowerConfig = 
          new PPHolonomicDriveController(
                new PIDConstants(1),
                new PIDConstants(1),
                MAX_OMEGA_RADIANS,
                DRIVE_BASE_RADIUS);

  /**
   * Generates configuration for the steering TalonFX.
   *
   * @param moduleNumber The module number for indexing into arrays.
   * @return The TalonFX configuration for the steering motor.
   */
  public TalonFXConfiguration steerConfig(int moduleNumber) {
    // Continuous wrap configuration for steering
    ClosedLoopGeneralConfigs continuousWrapConfig = new ClosedLoopGeneralConfigs();
    continuousWrapConfig.ContinuousWrap = true;

    return new TalonFXConfiguration()
            .withSlot0(
                    // PID slot configuration with proportional gain
                    new Slot0Configs()
                            .withKP(steer_kP))
            .withMotionMagic(
                    // Motion magic configuration for cruise velocity, acceleration, with expo
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(100 / steerRatio)
                            .withMotionMagicAcceleration((100 / steerRatio)/ .1)
                            .withMotionMagicExpo_kV(.12 * steerRatio)
                            .withMotionMagicExpo_kA(.1))
            .withClosedLoopGeneral(continuousWrapConfig)
            .withFeedback(
                    // Feedback configuration with rotor offset and remote CANcoder ID
                    new FeedbackConfigs()
                            .withFeedbackRotorOffset(STEERING_MODULE_OFFSET[moduleNumber])
                            .withFeedbackRemoteSensorID(CANCODER_ID[moduleNumber])
                            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
  }

  /**
   * Generates configuration for the throttle TalonFX.
   *
   * @return The TalonFX configuration for the throttle motor.
   */
  public TalonFXConfiguration throttleConfig() {
    return new TalonFXConfiguration()
            .withSlot0(
                    // PID slot configuration with proportional gain
                    new Slot0Configs().withKP(throttle_kP))
            .withCurrentLimits(
                    // Current limit configuration
                    new CurrentLimitsConfigs()
                            .withSupplyCurrentLimit(30)
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyTimeThreshold(.1))
            .withVoltage(
                    // Voltage limit configuration for forward and reverse voltages
                    new VoltageConfigs()
                            .withPeakForwardVoltage(10)
                            .withPeakReverseVoltage(-10));
  }

  /**
   * Generates configuration for the CANcoder.
   *
   * @param moduleNumber The module number for indexing into arrays.
   * @return The CANcoder configuration for the module.
   */
  public CANcoderConfiguration encoderConfig(int moduleNumber) {
    return new CANcoderConfiguration()
            .withMagnetSensor(
                    // Magnet sensor configuration with sensor range, direction, and offset
                    new MagnetSensorConfigs()
                            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                            .withMagnetOffset(STEERING_MODULE_OFFSET[moduleNumber]));
  }
}