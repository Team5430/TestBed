package com.team5430.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveModuleConstants {

  public SwerveModuleConstants() {}

  //max velocity of the robot as a whole in meters per second
  public double MAX_VELOCITY_MPS = 5;
  //Max angular velocity of the swerve modules 
  public double MAX_OMEGA_RADIANS = 10;
  //radius of the robot, from center of robot
  public double DRIVE_BASE_RADIUS = Units.inchesToMeters(12);


  // all arrays follow the order of A = 0, B = 1, C = 2, D = 3
  public double[] STEERING_MODULE_OFFSET = {0.083, 0.02954, .27416, .44789};

  public boolean[] MOTOR_INVERT = {true, false, true, false};

  protected Translation2d[] ModuleLocations = {
    new Translation2d(-0.267, -0.267), // back left A
    new Translation2d(0.267, 0.267), // front right B
    new Translation2d(-0.267, 0.267), // front left  C
    new Translation2d(0.267, -0.267) // back right D
  };

  public SwerveDriveKinematics Kinematics =
      new SwerveDriveKinematics(
          // Translation2d -> location of the graph in swerve module is Front is postive, Left is
          // positive
          // Measurement: Meters
          ModuleLocations);
}
