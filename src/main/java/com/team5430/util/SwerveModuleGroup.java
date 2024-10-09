package com.team5430.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleSupplier;

public class SwerveModuleGroup implements Sendable {

  private SwerveModule m_A;
  private SwerveModule m_B;
  private SwerveModule m_C;
  private SwerveModule m_D;

  SwerveModuleState state_A;
  SwerveModuleState state_B;
  SwerveModuleState state_C;
  SwerveModuleState state_D;

  private DoubleSupplier gyroscope;
  private double currentAngle;
  private double forward;
  private double sideways;
  private double angular;
  ChassisSpeeds speeds;
  private final StructArrayPublisher<SwerveModuleState> publisher;

  public SwerveModuleGroup(SwerveModule A, SwerveModule B, SwerveModule C, SwerveModule D) {
    m_A = A;
    m_B = B;
    m_C = C;
    m_D = D;
    publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    initStates();
  
  }
//Translation2d -> location of the graph in swerve module is  Front is postive, Left is positive 
//Measurement: Meters 
  Translation2d A_Location = new Translation2d(-0.267, -0.267); //back right
  Translation2d B_Location = new Translation2d(0.267, 0.267); //front left
  Translation2d C_Location = new Translation2d(0.267, -0.267); //back left
  Translation2d D_Location = new Translation2d(-0.267, 0.267); //front right

  //Log all Translation2d variables in Kinematic
  SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
    A_Location, B_Location, C_Location, D_Location);

    //ChassisSpeeds: 1 m/s front, 3m/s to left, 1.5 radians per seconds clockwise
    //NOT A CURRENT VELOCITY, MAX VELOCITY
        //converting chassis speeds to module states
        public void initStates(){
     state_A = new SwerveModuleState(m_A.getThrottle(), Rotation2d.fromDegrees(m_A.getHeading()));
     state_B = new SwerveModuleState(m_B.getThrottle(), Rotation2d.fromDegrees(m_B.getHeading()));
     state_C = new SwerveModuleState(m_C.getThrottle(), Rotation2d.fromDegrees(m_C.getHeading()));
     state_D = new SwerveModuleState(m_D.getThrottle(), Rotation2d.fromDegrees(m_D.getHeading()));
    

    speeds = m_Kinematics.toChassisSpeeds(
    state_A, state_B, state_C, state_D);
    
     forward = speeds.vxMetersPerSecond;
     sideways = speeds.vyMetersPerSecond;
     angular = speeds.omegaRadiansPerSecond;
  }
   




  public void setAngle(double input) {
    m_A.setAngle(input);
    m_B.setAngle(input);
    m_C.setAngle(input);
    m_D.setAngle(input);
  }


  public void setThrottle(double throttle) {
   //m_A.setThrottle(speeds.vxMetersPerSecond);
   m_A.setThrottle(throttle);
    m_B.setThrottle(-throttle);
    m_C.setThrottle(-throttle);
    m_D.setThrottle(throttle);
  }

  public void setAngleRatio(double ratio) {
    m_A.setAngleRatio(ratio);
    m_B.setAngleRatio(ratio);
    m_C.setAngleRatio(ratio);
    m_D.setAngleRatio(ratio);
    
  }

  public void setDriveRatio(double ratio) {
    m_A.setDriveRatio(ratio);
    m_B.setDriveRatio(ratio);
    m_C.setDriveRatio(ratio);
    m_A.setDriveRatio(ratio);
  }

  public void SetGains(double AngleMotorkP, double DriveMotorkP) {
    m_A.SetGains(AngleMotorkP, DriveMotorkP);
    m_B.SetGains(AngleMotorkP, DriveMotorkP);
    m_C.SetGains(AngleMotorkP, DriveMotorkP);
    m_D.SetGains(AngleMotorkP, DriveMotorkP);
  }

  public void setGyro(DoubleSupplier GyroscopeAngle) {
    gyroscope = GyroscopeAngle;
  }

  
   /** The bigger the input, smaller the output; meant to mimic breaking in a car */
   public double VariableSpeedDecline(Double input) {
    return 1 - input;
  }

  public void getGyroAngle() {
    currentAngle = gyroscope.getAsDouble();
  }

  public void Drive(double wantedAngle, double throttle, double thirdAxis, double Robotangle, double breaking) {

    // if turning within range of deadzone;
    if (thirdAxis > .3 || thirdAxis < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(thirdAxis, .3) + throttle;

      // adjust as needed; used to change Robotangle
      m_A.setThrottle(power/5  * VariableSpeedDecline(breaking));
      m_B.setThrottle(power/5  * VariableSpeedDecline(breaking));
      m_C.setThrottle(power/5  * VariableSpeedDecline(breaking));
      m_D.setThrottle(power/5  * VariableSpeedDecline(breaking));

      setAngle(Robotangle - currentAngle);
    } else {
      // when not turning
      setAngle(wantedAngle);
      setThrottle(throttle/5  * VariableSpeedDecline(breaking));
    }

  
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_A.initSendable(builder);
    m_B.initSendable(builder);
    m_C.initSendable(builder);
    m_D.initSendable(builder);
  }

  public void publishStates(){
    publisher.set(new SwerveModuleState[] {
      state_A,
      state_B,
      state_C,
      state_D
    });
  }
  


  
}