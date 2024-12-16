package com.team5430.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;


public class SimSwerveModule
{

  //THANKS TO BRONC BOTZ FOR THIS WORK OF ART
  // * Do note this is modified to include getting the change in module position as to use for a simulated gyroscope

  /**
   * Main timer to simulate the passage of time.
   */
  private final Timer             timer;
  /**
   * Time delta since last update
   */
  private       double            dt;
  /**
   * Fake motor position.
   */
  private       double            pos;
  /**
   * The fake speed of the previous state, used to calculate {@link SimSwerveModule#fakePos}.
   */
  private       double            speed;
  /**
   * Last time queried.
   */
  private       double            lastTime;

  private       double             lastPos;

  /**
   * Current simulated swerve module state.
   */
  private       SwerveModuleState state;
  private SwerveModulePosition last;



  /**
   * Create simulation class and initialize module at 0.
   */
  public SimSwerveModule()
  {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    speed = 0;
    pos = 0;
    dt = 0;
    lastPos = 0;
    last = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
  }

  /**
   * Update the position and state of the module. Called from {@link swervelib.SwerveModule#setDesiredState} function
   * when simulated.
   *
   * @param desiredState State the swerve module is set to.
   */
  public void updateStateAndPosition(SwerveModuleState desiredState)
  {
    //get time delta
    dt = timer.get() - lastTime;
    lastTime = timer.get();

    
    state = desiredState;
    speed = desiredState.speedMetersPerSecond;

    pos += (speed * dt);

    lastPos = pos;
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition()
  {

    return new SwerveModulePosition(pos, state.angle);
  }

  /**
   * get the change in the simulated swerve module position
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getModuleDelta(){

    var delta = new SwerveModulePosition(getPosition().distanceMeters - last.distanceMeters, state.angle);

     last = new SwerveModulePosition(lastPos, state.angle);

    return delta;

  }

  /**
   * Get the {@link SwerveModuleState} of the simulated module.
   *
   * @return {@link SwerveModuleState} of the simulated module.
   */
  public SwerveModuleState getState()
  {
    return state;
  }


}