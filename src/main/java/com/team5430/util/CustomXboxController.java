package com.team5430.util;

import edu.wpi.first.wpilibj.XboxController;

public class CustomXboxController extends XboxController {

    public enum Axis {
        X, Y
    }

    public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public int id;

        Button(int ID) {
            this.id = ID;
        }

    }

    public CustomXboxController(int port) {
        super(port);
    }    
    
    public void setRumble(boolean on) {
        this.setRumble(RumbleType.kBothRumble, on ? 1 : 0);
    }

    public double getLeftStickDirrectionDegrees(){
        double conversion =  Math.atan2(this.getLeftX(), -getLeftY());;
            return conversion;
    }
    
    public double getRightStickDirrectionDegrees(){
        double conversion =  Math.atan2(this.getRightX(), -getRightY());;
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