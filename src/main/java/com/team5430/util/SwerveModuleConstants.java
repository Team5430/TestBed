package com.team5430.util;

public class SwerveModuleConstants {

  public SwerveModuleConstants() {}

  public int SteeringMotorCANid;
  public int ThrottleMotorCANid;
  public int CANCoderCANid;
  public double SteeringGearRatio = 1;
  public double ThrottleGearRatio = 8.14;
  public double SteeringkP  = .1;
  public double ThrottlekP = .5;
  public String Name;
  public static final double MODULE_STEER_OFFSET = 0;
  }
