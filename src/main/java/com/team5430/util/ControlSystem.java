package com.team5430.util;

/*
 *
 * A standard for subsystem to implement if they are some sort of mechanism,
 * as to be able to quickly manipulate them in different situations.
 *  
 */
public interface ControlSystem {
    
     public boolean hasEmergency = true; 

     default void Stop(){}

     default void configureTest(){}

     default void configurePeriodic(){}

}
