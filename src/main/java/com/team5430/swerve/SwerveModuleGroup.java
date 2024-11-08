package com.team5430.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class SwerveModuleGroup {

  // max is 4 swerve modules; accounted for array
  protected SwerveModule[] swerveModules = new SwerveModule[4];
  protected int moduleCount;

  protected SwerveModuleConstants constants;

  private final StructArrayPublisher<SwerveModuleState> StatePublisher;

  private final StructArrayPublisher<SwerveModulePosition> PositionPublisher;

  /**
   * Modular Swerve creation, can be used to create up to 4 modules at a time. NOTE: Consider
   * reserving CANids 0-8 for motors and CANCoders
   *
   * <pre>Supports:
   *   CANCoders
   *   TalonFX based motors
   *   </pre>
   *
   * <p>An example use case would be a ModuleCount of 3, where
   *
   * <pre>Module_1              Module_2:            Module_3:
   *
   *  SteeringCANid: 0     SteeringCANid: 2    SteeringCANid: 4
   *  ThrottleCANid: 1     ThrottleCANid: 3    ThrottleCANid: 5
   *  CANCoderCANid: 0     CANCoderCANid: 1    CANCoderCANid: 2</pre>
   *
   * <p>to configure this to your use case, utilise SwerveModuleConstants
   *
   * @param ModuleCount Allows creation of up to 4 SwerveModules, based on your given config
   * @see com.team5430.swerve.SwerveModuleConstants
   */
  public SwerveModuleGroup(int ModuleCount, SwerveModuleConstants config) {

    moduleCount = ModuleCount;

    for (int i = 0; i < moduleCount; i++) {
      swerveModules[i] = new SwerveModule(i * 2, i * 2 + 1, i, config.STEERING_MODULE_OFFSET[i]);
      swerveModules[i].invertThrottle(config.MOTOR_INVERT[i]);
    }
    // set config
    constants = config;

    // Start publishing an array of module states with the "/SwerveStates" key
    StatePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct)
            .publish();

    PositionPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwervePositions", SwerveModulePosition.struct)
            .publish();
    // **next step! */    SwerveDrivePoseEstimator t = new SwerveDrivePoseEstimator(m_Kinematics,
    // null, null, null)
  }

  /** Set Module States to desired state */
  public void SetStates(SwerveModuleState... currentStates) {
    // Prevent Speed from surpassing maxSpeed
    SwerveDriveKinematics.desaturateWheelSpeeds(currentStates, constants.MAX_VELOCITY_MPS);

    // apply states in a for Loop.
    for (int i = 0; i < moduleCount; i++) {
      swerveModules[i].setState(currentStates[i]);
    }
  }

  /** forward is relative to the robots forward, classic */
  public void RobotRelativeDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = constants.Kinematics.toSwerveModuleStates(speeds);
    SetStates(states);
  }

  /** drive with a gyroscope to keep heading to field */
  public void FieldCentricDrive(ChassisSpeeds speeds, Rotation2d robotAngle) {
    RobotRelativeDrive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle));
  }

  /**
   * @return the angle and velocity of the robot
   */
  public ChassisSpeeds getCurrentSpeeds() {

    return constants.Kinematics.toChassisSpeeds(getStates(true));
  }

  /** STOP!! */
  public void Stop() {

    for (SwerveModule s : swerveModules) {
      s.Stop();
    }
  }

  // distance and angle
  public SwerveModulePosition[] getPositions(boolean refresh) {

    return new SwerveModulePosition[] {
      swerveModules[0].getPosition(refresh),
      swerveModules[1].getPosition(refresh),
      swerveModules[2].getPosition(refresh),
      swerveModules[3].getPosition(refresh),
    };
  }

  // velocity and angle
  public SwerveModuleState[] getStates(boolean refresh) {

    return new SwerveModuleState[] {
      swerveModules[0].getState(refresh),
      swerveModules[1].getState(refresh),
      swerveModules[2].getState(refresh),
      swerveModules[3].getState(refresh),
    };
  }

  public void publishData() {
    StatePublisher.set(getStates(true));
    PositionPublisher.set(getPositions(true));
  }
}
