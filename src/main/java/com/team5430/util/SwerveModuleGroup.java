package com.team5430.util;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;



public class SwerveModuleGroup  {

//max is 4 swerve modules; accounted for array
  private SwerveModule[] swerveModules =  new SwerveModule[4];
  private int moduleCount;

  private static SwerveDriveKinematics m_Kinematics;
  private static SwerveModuleState[] m_states;
  

  private final StructArrayPublisher<SwerveModuleState> publisher;

  public enum DriveStyle {
    FIELD_CENTRIC,
    RELATIVE
  }


  
/**
   * Modular Swerve creation, can be used to create up to 4 modules at a time. NOTE: Consider
   * reserving CANids 0-8 for motors and CANCoders
   *
   * <pre>Supports:
   *   CANCoders
   *   TalonFX based motors
   *   </pre>
   * <p>
   * An example use case would be a ModuleCount of 3, where
   *
   * <pre>Module_1              Module_2:             Module_3:
   *
   *  SteeringCANid: 0     SteeringCANid: 2     SteeringCANid: 4
   *  ThrottleCANid: 1     ThrottleCANid: 3     ThrottleCANid: 5
   *  CANCoderCANid: 0     CANCoderCANid: 1     CANCoderCANid: 2</pre>
   * <p>
   * to configure this to your use case, utilise SwerveModuleConstants
   *
   * @param ModuleCount Allows creation of up to 4 SwerveModules, based on your given config
   * @see com.team5430.util.SwerveModuleConstants
   */
  public SwerveModuleGroup(int ModuleCount, SwerveModuleConstants config) {

moduleCount = ModuleCount;


    for(int i= 0; i < moduleCount; i++){
      swerveModules[i] = new SwerveModule(i * 2, i * 2 + 1, i, config.STEERING_MODULE_OFFSET[i]);
      swerveModules[i].invertThrottle(config.MOTOR_INVERT[i]);
    }
    //set Kinematics
    m_Kinematics = config.Kinematics;

        // Start publishing an array of module states with the "/SwerveStates" key
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();


    
//**next step! */    SwerveDrivePoseEstimator t = new SwerveDrivePoseEstimator(m_Kinematics, null, null, null)
  }
   
   /** The bigger the input, smaller the output; meant to mimic breaking in a car */  
   public double VariableSpeedDecline(double input) {
    return 1 - input;
  }

  public void DriveToDistance(double distance){
    for(int i = 0; i < moduleCount; i++){
      swerveModules[i].DriveToDistance(distance);
    }
  }
  public void SetStates(SwerveModuleState... currentStates){  
  //Prevent Speed from surpassing maxSpeed
    SwerveDriveKinematics.desaturateWheelSpeeds(currentStates, 12);

    //apply states in a for Loop.
    for(int i = 0; i < moduleCount; i++){
      swerveModules[i].setState(currentStates[i]);
    }

  }
  
  public void Drive(ChassisSpeeds speeds){
    SwerveModuleState states[] = m_Kinematics.toSwerveModuleStates(speeds);
    SetStates(states);
    m_states = states;
  }

  public void FieldCentricDrive(ChassisSpeeds speeds, Rotation2d robotAngle){
    Drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle)
    );
  }
/**STOP!! */
  public void Stop(){

    for(SwerveModule s : swerveModules){
      s.Stop();
    }

  }

  
  public void publishData() {
      publisher.set(m_states);
    }
  }

