package com.team5430.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;

public class booleans {

  private booleans() {}

  public enum RobotType{
    SIM_ROBOT,
    REAL_ROBOT,
  }

  public static BooleanSupplier isBlue() {
    return () -> {
      var alliance = DriverStation.getAlliance();
      return alliance.filter(color -> color == DriverStation.Alliance.Blue).isPresent();
    };
  }

  public static BooleanSupplier isTeleop() {
    return DriverStation::isTeleop;
  }

  public static BooleanSupplier isAutonomous() {
    return DriverStation::isAutonomous;
  }

  public static RobotType getRobot() {
    if(!Robot.isReal()){
      return RobotType.SIM_ROBOT;
    }
      return RobotType.REAL_ROBOT;
  }
}
