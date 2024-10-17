package com.team5430.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveModuleConstants {

  public SwerveModuleConstants() {}

  public double[] STEERING_MODULE_OFFSET ={
    0.431,
    0.15,
    .03,
    0
  };

  public boolean[] MOTOR_INVERT = {
    false,
    true,
    true,
    false
  };

  private Translation2d[] ModuleLocations = {
   new Translation2d(-0.267, -0.267), //back right A
   new Translation2d(0.267, 0.267), //front left B 
   new Translation2d(-0.267, 0.267), //back left C
   new Translation2d(0.267, -0.267) //front right D
  };


public SwerveDriveKinematics Kinematics = new SwerveDriveKinematics( 
  //Translation2d -> location of the graph in swerve module is  Front is postive, Left is positive 
//Measurement: Meters 
ModuleLocations
    );
  }
