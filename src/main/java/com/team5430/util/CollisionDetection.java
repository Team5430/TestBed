// Inspired by NavX-Mxp example for collision detection
package com.team5430.util;

import com.kauailabs.navx.frc.AHRS;
import java.util.function.BooleanSupplier;

public class CollisionDetection {

  private AHRS gyroscope;
  // threshold to be tuned for robot
  double CollisionThreshold = .3f;

  double PreviousLinearXAccel;

  double PreviousLinearYAccel;

  public CollisionDetection(AHRS navx_mxp) {
    gyroscope = navx_mxp;
  }

  // use if needed for triggers
  public BooleanSupplier getDetection = () -> CollisionDetected();

  public boolean CollisionDetected() {
    // calculate jerk and see utilise any spikes too come out as true
    double LinearXAccel = gyroscope.getWorldLinearAccelX();
    double currentJerkX = LinearXAccel - PreviousLinearXAccel;
    PreviousLinearXAccel = LinearXAccel;

    double LinearYAccel = gyroscope.getWorldLinearAccelY();
    double currentJerkY = LinearYAccel - PreviousLinearYAccel;
    PreviousLinearYAccel = LinearYAccel;

    if ((Math.abs(currentJerkX) > CollisionThreshold)
        || (Math.abs(currentJerkY) > CollisionThreshold)) {
      return true;
    } else {
      return false;
    }
  }
}
