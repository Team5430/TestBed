package com.team5430.control;

/*
 *
 * A standard for subsystem to implement if they are some sort of mechanism,
 * as to be able to quickly manipulate them in different situations.
 *  
 */
public interface ControlSystem {
    
     public boolean hasEmergency = true; 

     default void Stop(){}

     default boolean configureTest(){
               return true;
          }

     default void configurePeriodic(){}

}
