package com.team5430.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleGroup {

  // Maximum of 4 swerve modules; accounted for array
  protected SwerveModule[] swerveModules = new SwerveModule[4];
  protected int moduleCount;
  protected SwerveModuleConstants constants;

  /**
   * Modular Swerve creation, can be used to create up to 4 modules at a time.
   * NOTE: Consider reserving CANids 0-8 for motors and CANCoders
   *
   * <pre>Supports:
   *   CANCoders
   *   TalonFX based motors
   * </pre>
   *
   * <p>An example use case would be a ModuleCount of 3, where
   *
   * <pre>
   * Module_1              Module_2:            Module_3:
   *
   *  SteeringCANid: 0     SteeringCANid: 2    SteeringCANid: 4
   *  ThrottleCANid: 1     ThrottleCANid: 3    ThrottleCANid: 5
   *  CANCoderCANid: 0     CANCoderCANid: 1    CANCoderCANid: 2
   * </pre>
   *
   * <p>to configure this to your use case, utilize SwerveModuleConstants
   *
   * @param ModuleCount Allows creation of up to 4 SwerveModules, based on your given config
   * @param config Configuration for the swerve modules
   * @see com.team5430.swerve.SwerveModuleConstants
   */
  public SwerveModuleGroup(int ModuleCount, SwerveModuleConstants config) {
    moduleCount = ModuleCount;
    for (int i = 0; i < moduleCount; i++) {
      swerveModules[i] = new SwerveModule(i, config);
    }
    constants = config;
  }

  /** Set Module States to desired state */
  public void setStates(SwerveModuleState... currentStates) {
    // Prevent speed from surpassing max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(currentStates, constants.MAX_VELOCITY_MPS);
    // Apply states in a loop
    for (int i = 0; i < moduleCount; i++) {
      swerveModules[i].setState(currentStates[i]);
    }
  }

  /** Forward is relative to the robot's forward */
  public void robotRelativeDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = constants.Kinematics.toSwerveModuleStates(speeds);
    setStates(states);
  }

  /** Drive with a gyroscope to keep heading relative to the field */
  public void fieldCentricDrive(ChassisSpeeds speeds, Rotation2d robotAngle) {
    robotRelativeDrive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle));
  }

  /**
   * Get the current speeds of the robot.
   *
   * @return the angle and velocity of the robot
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return constants.Kinematics.toChassisSpeeds(getStates(true));
  }

  /** Stop all swerve modules */
  public void Stop() {
    for (SwerveModule s : swerveModules) {
      s.Stop();
    }
  }

  /** Get positions of all modules */
  public SwerveModulePosition[] getPositions(boolean refresh) {
    return new SwerveModulePosition[] {
            swerveModules[0].getPosition(refresh),
            swerveModules[1].getPosition(refresh),
            swerveModules[2].getPosition(refresh),
            swerveModules[3].getPosition(refresh)
    };
  }

  /** Get states of all modules */
  public SwerveModuleState[] getStates(boolean refresh) {
    return new SwerveModuleState[] {
            swerveModules[0].getState(refresh),
            swerveModules[1].getState(refresh),
            swerveModules[2].getState(refresh),
            swerveModules[3].getState(refresh)
    };
  }

 

}
