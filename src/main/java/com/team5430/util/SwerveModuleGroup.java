package com.team5430.util;

import org.ejml.equation.VariableDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModuleGroup {

  private SwerveModule m_A;
  private SwerveModule m_B;
  private SwerveModule m_C;
  private SwerveModule m_D;
  public ShuffleboardTab DataTab = Shuffleboard.getTab("Swerve Chassis");
  private double lastAngle = 0;

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

  public double VariableSpeedDecline(Double input) {

    double breaking = 1 - input;
    return breaking;
  }


  public void Drive(
      double wantedAngle, double throttle, double thirdAxis, double Robotangle, double breaking) {

    // if turning within range of deadzone;

    if (thirdAxis > .3 || thirdAxis < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(thirdAxis, .3) + throttle;

      // adjust as needed; use to rotate
      m_A.setThrottle(-power / 2 * VariableSpeedDecline(breaking));
      m_B.setThrottle(-power / 2 * VariableSpeedDecline(breaking));
      m_C.setThrottle(power / 2 * VariableSpeedDecline(breaking));
      m_D.setThrottle(power / 2 * VariableSpeedDecline(breaking));

      setAngle(Robotangle);
    } else {
      // when not turning
      setAngle(wantedAngle);
      setThrottle(MathUtil.applyDeadband(throttle, .2) * .1 *  VariableSpeedDecline(breaking));
    }
  }

  public void addTab() {
    // for shuffleboard
    Shuffleboard.getTab("Swerve Chassis").add(m_A).withSize(2, 3);

    Shuffleboard.getTab("Swerve Chassis").add(m_B).withSize(2, 3);

    Shuffleboard.getTab("Swerve Chassis").add(m_C).withSize(2, 3);

    Shuffleboard.getTab("Swerve Chassis").add(m_D).withSize(2, 3);
  }
}
