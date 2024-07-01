package com.team5430.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModuleGroup {

  private SwerveModule m_A;
  private SwerveModule m_B;
  private SwerveModule m_C;
  private SwerveModule m_D;
  private double lastAngle;
  public ShuffleboardTab DataTab = Shuffleboard.getTab("Swerve Chassis");

  // TODO add SetPose2D, possibly SwerveModuleType as to configure chassis no matter the use case,
  // filters for gyro to combat noise depending on data,
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

  // **the wheel will go to the position that is greater than 0.2, otherwise stop power when less
  // than or equal to*/
  private double deadzone(double angle, double power) {
    // If the input given is less than 0.3 then it will return last value saved
    if (power < -0.3) {
      lastAngle = angle;
    }
    return lastAngle;
  }

  public double VariableSpeedDecline(Double input) {
    return 1 - input;
  }

  /**
   * @param wantedAngle sets modules heading angle to given value
   * @param throttle sets velocity with a max set within SwerveModule
   * @param rotation2d axis to control rotation of Robot
   * @param robotAngle usually a gyroscope to feed the robots heading
   * @see com.team5430.util.SwerveModule
   */
  public void Drive(double wantedAngle, double throttle, double rotation2d, double robotAngle) {

    // if turning within range of deadzone;

    if (rotation2d > .3 || rotation2d < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(rotation2d, .3) + throttle;

      // adjust as needed; use to rotate
      m_A.setThrottle(-power / 2);
      m_B.setThrottle(-power / 2);
      m_C.setThrottle(power / 2);
      m_D.setThrottle(power / 2);

      setAngle(robotAngle);
    } else {
      // when not turning
      setAngle(deadzone(wantedAngle, throttle));
      setThrottle(MathUtil.applyDeadband(throttle, .2) * .1);
    }
  }

  /**
   * @param wantedAngle sets modules heading angle to given value
   * @param throttle sets velocity with a max set within SwerveModule
   * @param rotation2d axis to control rotation of Robot
   * @param robotAngle usually a gyroscope to feed the robots heading
   * @param breaking axis that controls a decreasing multiplier
   * @see com.team5430.util.SwerveModule
   */
  public void Drive(
      double wantedAngle, double throttle, double rotation2d, double robotAngle, double breaking) {

    // if turning within range of deadzone;

    if (rotation2d > .3 || rotation2d < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(rotation2d, .3) + throttle;

      // adjust as needed; use to rotate
      m_A.setThrottle(-power / 2 * VariableSpeedDecline(breaking));
      m_B.setThrottle(-power / 2 * VariableSpeedDecline(breaking));
      m_C.setThrottle(power / 2 * VariableSpeedDecline(breaking));
      m_D.setThrottle(power / 2 * VariableSpeedDecline(breaking));

      setAngle(robotAngle);
    } else {
      // when not turning
      setAngle(deadzone(wantedAngle, throttle));
      setThrottle(MathUtil.applyDeadband(throttle, .2) * .1 * VariableSpeedDecline(breaking));
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
