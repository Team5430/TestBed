//Inspired by NavX-Mxp example for collision detection
package com.team5430.util;

import com.kauailabs.navx.frc.AHRS;

public class CollisionDetection {

private AHRS gyroscope; 

double CollisionThreshold = .5f;

double PreviousLinearXAccel;

double PreviousLinearYAccel;

    public CollisionDetection(AHRS navx_mxp){
        gyroscope = navx_mxp;
    }
 
    public boolean CollisionDetected(){

        double LinearXAccel = gyroscope.getWorldLinearAccelX();
        double currentJerkX = LinearXAccel - PreviousLinearXAccel;
        PreviousLinearXAccel = LinearXAccel;
        double LinearYaccel = gyroscope.getWorldLinearAccelY();
        double currentJerkY = LinearYaccel - PreviousLinearYAccel;
        PreviousLinearYAccel = LinearYaccel;

        if ( ( Math.abs(currentJerkX) > CollisionThreshold ) ||
            (Math.abs(currentJerkY) > CollisionThreshold) ) {
       return true;
   }
        return false; 
    }
}
