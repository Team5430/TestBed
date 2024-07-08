package com.team5430.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveModuleGroup {

  private final SwerveModule[] swerveModules;
  private double lastAngle;
  private String Title;
  // TODO add SetPose2D, Velocity for SwerveModule in general; setThrottle

  /**
   * Modular Swerve creation, can be used to create up to 4 modules at a time. NOTE: Consider
   * reserving CANids 0-11 for motors and CANCoders
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
   *  CANCoderCANid: 8     CANCoderCANid: 9     CANCoderCANid: 10</pre>
   * <p>
   * to configure this to your use case, utilise SwerveModuleConstants
   *
   * @param ModuleCount Allows creation of up to 4 SwerveModules, based on your given config
   * @see com.team5430.util.SwerveModuleConstants
   */
  public SwerveModuleGroup(int ModuleCount, SwerveModuleConstants config) {
    Title = config.Name;
    // minus 1 as arrays start at 0; would create 1 extra if not placed
    swerveModules = new SwerveModule[ModuleCount - 1];
    //
    for (int i = 0; i < swerveModules.length; i++) {
      // allows for synchronized creation of modules in order starting from CANids 0,1 for one
      // module,
      // following more CANids for more modules, and then reserving 8 9 10 11 for CANCoders
      swerveModules[i] = new SwerveModule(i * 2, i * 2 + 1, i + 8, config);
    }
  }

  /**
   * Set heading angle for all modules to follow
   */
  public void setSteering(double input) {
    for (SwerveModule s : swerveModules) {
      s.setSteering(input);
    }
  }

  /** Set throttle to all Swerve Modules */
  public void setThrottle(double throttle) {
    for (SwerveModule s : swerveModules) {
      s.setThrottle(throttle);
    }
  }

  // **the wheel will go to the position that is greater than 0.2, otherwise stop power when less
  // than or equal to*/
  private double deadzone(double Steering, double power) {
    // If the input given is less than 0.3 then it will return last value saved
    if (power < -0.3) {
      lastAngle = Steering;
    }
    return lastAngle;
  }

  /** The bigger the input, smaller the output; meant to mimic breaking in a car */
  public double VariableSpeedDecline(Double input) {
    return 1 - input;
  }

  /**
   * @param wantedSteering sets modules heading Steering to given value
   * @param throttle sets velocity with a max set within SwerveModule
   * @param rotation2d axis to control rotation of Robot
   * @param robotSteering usually a gyroscope to feed the robots heading
   * @see com.team5430.util.SwerveModule
   */
  public void Drive(
          double wantedSteering, double throttle, double rotation2d, double robotSteering) {

    // if turning within range of deadzone;

    if (rotation2d > .3 || rotation2d < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(rotation2d, .3) + throttle;

      // set to opposite powers for rotation, adjust multipliers or dividends as needed
      // NOTE: offset so 1 and 3 are given same power, and 0 and 2 their own respective
      for (int i = 0; i < 3; i = i + 2) {
        swerveModules[i].setThrottle(-power / 2);
      }
      for (int i = 1; i < 4; i = i + 2) {
        swerveModules[i].setThrottle(power / 2);
      }

      setSteering(robotSteering);
    } else {
      // when not turning
      setSteering(deadzone(wantedSteering, throttle));
      setThrottle(MathUtil.applyDeadband(throttle, .2) * .1);
    }
  }

  /**
   * @param wantedSteering sets modules heading Steering to given value
   * @param throttle sets velocity with a max set within SwerveModule
   * @param rotation2d axis to control rotation of Robot
   * @param robotSteering usually a gyroscope to feed the robots heading
   * @param breaking axis that controls a decreasing multiplier
   * @see com.team5430.util.SwerveModule
   */
  public void Drive(
          double wantedSteering,
          double throttle,
          double rotation2d,
      double robotSteering,
      double breaking) {

    // if turning within range of deadzone;

    if (rotation2d > .3 || rotation2d < -.3) {
      // used to drive while turning
      double power = MathUtil.applyDeadband(rotation2d, .3) + throttle;

      // set to opposite powers for rotation, adjust multipliers or dividends as needed
      for (int i = 0; i < 3; i = i + 2) {
        swerveModules[i].setThrottle(-power / 2);
      }
      for (int i = 1; i < 4; i = i + 2) {
        swerveModules[i].setThrottle(power / 2);
      }

      setSteering(robotSteering);
    } else {
      // when not turning
      setSteering(deadzone(wantedSteering, throttle));
      setThrottle(MathUtil.applyDeadband(throttle, .2) * .1 * VariableSpeedDecline(breaking));
    }
  }

  public void addTab() {
    // add widgets for every swerveModule in ShuffleBoard
    for (SwerveModule s : swerveModules) {
      Shuffleboard.getTab(Title).add(s).withSize(2, 3);
      }
  }
}
