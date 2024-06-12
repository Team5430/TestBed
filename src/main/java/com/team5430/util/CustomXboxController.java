package com.team5430.util;

import edu.wpi.first.wpilibj.XboxController;

public class CustomXboxController extends XboxController {

    public CustomXboxController(int port) {
        super(port);
    }    
    
    public void setRumble(boolean on) {
        this.setRumble(RumbleType.kBothRumble, on ? 1 : 0);
    }

    public double getLeftStickDirrectionDegrees(){
        double conversion =  Math.toDegrees(Math.atan2(this.getLeftX(), -getLeftY()));
            return conversion;
    }
    
    public double getRightStickDirrectionDegrees(){
        double conversion =  Math.toDegrees(Math.atan2(this.getRightX(), -getRightY()));
           return conversion;
    }
    
    public double getLeftMagnitude(){
         return Math.hypot(getLeftX(), getLeftY());
    }

    public double getRightMagnitude(){
        return Math.hypot(getRightX(), getRightY());
    }

    public XboxController getController() {
        return this;
    }
}