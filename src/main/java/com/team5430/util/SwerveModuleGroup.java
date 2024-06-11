package com.team5430.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleSupplier;

public class SwerveModuleGroup implements Sendable {

  private SwerveModule m_A;
  private SwerveModule m_B;
  private SwerveModule m_C;
  private SwerveModule m_D;
  private DoubleSupplier gyroscope;
  private double currentAngle;

  public SwerveModuleGroup(SwerveModule A, SwerveModule B, SwerveModule C, SwerveModule D) {
    m_A = A;
    m_B = B;
    m_C = C;
    m_D = D;
  }

  public void SwerveModuleType() {}

  public void setAngle(double input) {
    m_A.setAngle(input);
    m_B.setAngle(input);
    m_C.setAngle(input);
    m_D.setAngle(input);
  }

  public void setThrottle(double throttle) {
    m_A.setThrottle(throttle);
    m_B.setThrottle(throttle);
    m_C.setThrottle(throttle);
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

  public void getGyroAngle() {
    currentAngle = gyroscope.getAsDouble();
  }

  public void Drive(double wantedAngle, double throttle, double thirdAxis, double Robotangle) {

    // if turning within range of deadzone;

    if (thirdAxis > .3 || thirdAxis < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(thirdAxis, .3) + throttle;

      // adjust as needed; used to change Robotangle
      setThrottle(power / 2);

      setAngle(Robotangle - currentAngle);
    } else {
      // when not turning
      setAngle(wantedAngle);
      setThrottle(throttle);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_A.initSendable(builder);
    m_B.initSendable(builder);
    m_C.initSendable(builder);
    m_D.initSendable(builder);
  }

  
}
