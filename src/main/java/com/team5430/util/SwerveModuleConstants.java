package com.team5430.util;

public class SwerveModuleConstants {

  public SwerveModuleConstants() {}

  public int SteeringMotorCANid;
  public int ThrottleMotorCANid;
  public int CANCoderCANid;
  public double SteeringGearRatio = 1;
  public double ThrottleGearRatio = 8.14;
  public double SteeringkP  = .95;
  public double ThrottlekP = .5;
  public String Name;
  public double[] STEERING_MODULE_OFFSET ={
    0.387,
    0.265,
    .03,
    0
  };
  }
