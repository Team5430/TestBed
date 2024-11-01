package com.team5430.util;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BooleanSupplier;

public class booleans {

    private booleans(){}

    public static BooleanSupplier isBlue(){
        return () ->
        {
            var alliance = DriverStation.getAlliance();
            return alliance.filter(color -> color == DriverStation.Alliance.Blue).isPresent();
        };
    }
}
