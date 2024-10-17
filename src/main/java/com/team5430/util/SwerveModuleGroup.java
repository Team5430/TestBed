package com.team5430.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;



public class SwerveModuleGroup implements Sendable {

//max is 4 swerve modules; accounted for array
  private SwerveModule[] swerveModules =  new SwerveModule[4];
  private int moduleCount;
  private AHRS _gyro;


  private double headingAngle;
  private double currentAngle;
  /* 
  private double forward;
  private double sideways;
  private double angular;
  private ChassisSpeeds speeds;
  private SwerveDriveKinematics m_Kinematics;
  private SwerveModuleState[] m_States;
  private SwerveModulePosition[] m_Positions;
  */
  
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
    }
  
    _gyro = new AHRS(Port.kMXP);
 
    //Translation2d -> location of the graph in swerve module is  Front is postive, Left is positive 
//Measurement: Meters 
 /*  Translation2d A_Location = new Translation2d(-0.267, -0.267); //back right
  Translation2d B_Location = new Translation2d(0.267, 0.267); //front left
  Translation2d C_Location = new Translation2d(0.267, -0.267); //back left
  Translation2d D_Location = new Translation2d(-0.267, 0.267); //front right


  speeds = m_Kinematics.toChassisSpeeds(
    );
    
     forward = speeds.vxMetersPerSecond;
     sideways = speeds.vyMetersPerSecond;
     angular = speeds.omegaRadiansPerSecond;

     m_Kinematics = new SwerveDriveKinematics(
    A_Location, B_Location, C_Location, D_Location);


  */
  }

  public void setAngle(double input) {
    
   for(SwerveModule s : swerveModules){
    s.setAngle(input);
   }
}


  public void setThrottle(double throttle) {

 for(SwerveModule s : swerveModules){
  s.setThrottle(throttle);
 }
  }

  public void setGyro() {
    headingAngle =_gyro.getAngle();
  }

  
   /** The bigger the input, smaller the output; meant to mimic breaking in a car */
   public double VariableSpeedDecline(Double input) {
    return 1 - input;
  }

  public double getGyroAngle() {
    return headingAngle;
  }

  public void Drive(double wantedAngle, double throttle, double thirdAxis, double Robotangle, double breaking) {
    
    // if turning within range of deadzone;
    if ((thirdAxis > .3 || thirdAxis < -.3) ) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(thirdAxis, .3) + throttle;

      //adjust as needed for turning speed
            setThrottle(power/10  * VariableSpeedDecline(breaking));

      setAngle(Robotangle - headingAngle); 
    } else {
      // when not turning
      setAngle(wantedAngle);
      
      swerveModules[0].setThrottle(throttle/10 * VariableSpeedDecline(breaking));
      swerveModules[1].setThrottle(-throttle/10 * VariableSpeedDecline(breaking));
      swerveModules[2].setThrottle(-throttle/10 * VariableSpeedDecline(breaking));
      swerveModules[3].setThrottle(throttle/10 * VariableSpeedDecline(breaking));
    //save angle
      setGyro();
      
    }

  
  }

  @Override
  public void initSendable(SendableBuilder builder) {
for(SwerveModule s : swerveModules){
  s.initSendable(builder);
}
  }

 
  


  
}