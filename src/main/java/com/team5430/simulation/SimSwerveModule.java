package com.team5430.simulation;


import com.team5430.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimSwerveModule {

  //TODO: look over; review
    SwerveModuleConstants constants = new SwerveModuleConstants();

    protected DCMotorSim steerSim
        = new DCMotorSim(DCMotor.getFalcon500(1), constants.steerRatio, 0.004096955);

    protected FlywheelSim throttleSim
        = new FlywheelSim(DCMotor.getKrakenX60(1), constants.throttleRatio, 0.025);

    private double steerAngle = 0.0;
    private double throttleSpeed = 0.0;
    private double throttlePos = 0.0;

  private SwerveModulePosition position = new SwerveModulePosition();
  private SwerveModuleState state = new SwerveModuleState();

    public SimSwerveModule(){}

  public void setState(SwerveModuleState desiredState) {
    // Set steer angle and throttle speed based on the desired state
    this.state = desiredState;
    this.setSteerAngle(desiredState.angle.getDegrees());
    this.setThrottleSpeed(desiredState.speedMetersPerSecond);
  }

  public SwerveModuleState getState(boolean refresh) {
    if (refresh) {
      this.state.angle = new Rotation2d(getSteerAngle());
      this.state.speedMetersPerSecond = getThrottleVelocityRadPerSec();
    }
    return this.state;
  }

  //TODO: fix distanceMeters
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      this.position.angle = new Rotation2d(getSteerAngle());
      this.position.distanceMeters = throttlePos;
    }
    return this.position;
  }
//set and get are inconsistent BEWARE
    public void setSteerAngle(double angle) {
        this.steerAngle = angle;
    }

    public double getSteerAngle() {
        return this.steerAngle;
    }

    public void setThrottleSpeed(double speed) {
        this.throttleSpeed = speed;
    }

    public double getThrottleSpeed() {
        return this.throttleSpeed;
    }

    /** Run periodically to update the simulation*/
    public void updateSim(double dt) {
        // Update the motors using input speed
        steerSim.setInputVoltage(throttleSpeed);
        steerSim.update(dt);

        throttleSim.setInputVoltage(throttleSpeed);
        throttleSim.update(dt);

        double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * .02;
        throttlePos += angleDiffRad;
        throttlePos = throttlePos + angleDiffRad;
    }

    //position values kind of suck for simulating a Swerve module; use saved {@code steerAngle} instead
    //(most likely user fault thought, so if you can try to make it work)
    public double getSteerAngularPositionRotations() {
        return steerSim.getAngularPositionRad();
    }

    public double getPos() {
      
        return steerSim.getAngularPositionRad() * Math.PI;
    }

    //these are `
    public double getSteerVelocityRadPerSec() {
        return steerSim.getAngularVelocityRadPerSec();
    }

    public double getThrottleVelocityRadPerSec() {
        return throttleSim.getAngularVelocityRadPerSec();
    }


}

   