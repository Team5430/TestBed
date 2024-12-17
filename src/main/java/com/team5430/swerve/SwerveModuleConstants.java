package com.team5430.swerve;

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
}