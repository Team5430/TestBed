package com.team5430.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;



public class SwerveModuleGroup {

  private SwerveModule m_A;
  private SwerveModule m_B;
  private SwerveModule m_C;
  private SwerveModule m_D;
  private DoubleSupplier gyroscope;
  private double currentAngle;
  private double angleInput = 1 / 360;

  public SwerveModuleGroup(SwerveModule A, SwerveModule B, SwerveModule C, SwerveModule D) {
    m_A = A;
    m_B = B;
    m_C = C;
    m_D = D;
  }

  public SwerveModuleGroup(SwerveModule A, SwerveModule B, SwerveModule C) {
    m_A = A;
    m_B = B;
    m_C = C;
  }

  public SwerveModuleGroup(SwerveModule A, SwerveModule B) {
    m_A = A;
    m_B = B;
  }

  public void SwerveModuleType() {}

  public void setAngle(double input) {
    m_A.setAngle(input);
    m_B.setAngle(input);

  }

  public void setThrottle(double throttle) {
    m_A.setThrottle(throttle);
    m_B.setThrottle(throttle);
 
  }

  public void setAngleRatio(double ratio) {
    m_A.setAngleRatio(ratio);
    m_B.setAngleRatio(ratio);
 
  }

  public void setDriveRatio(double ratio) {
    m_A.setDriveRatio(ratio);
    m_B.setDriveRatio(ratio);
   
  }

  public void SetGains(double AngleMotorkP, double DriveMotorkP) {
    m_A.SetGains(AngleMotorkP, DriveMotorkP);
    m_B.SetGains(AngleMotorkP, DriveMotorkP);

  }

  public void setGyro(DoubleSupplier GyroscopeAngle) {
    gyroscope = GyroscopeAngle;
  }

  public void getGyroAngle() {
    currentAngle = gyroscope.getAsDouble();
  }

  public void Drive(double wantedAngle, double throttle, double thirdAxis, double Robotangle) {
    // if turning within range of deadzone;
    
    currentAngle = Robotangle; 

    if (thirdAxis > .3 || thirdAxis < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(thirdAxis, .3) + throttle;

      // adjust as needed; used to change Robotangle
      m_A.setThrottle(power);
      m_B.setThrottle(power);
    

      double headingAngle = currentAngle - Robotangle;

      setAngle(headingAngle);
    } else {
      // when not turning
      setAngle(wantedAngle);
      setThrottle(throttle);
    }
  
  }

}
