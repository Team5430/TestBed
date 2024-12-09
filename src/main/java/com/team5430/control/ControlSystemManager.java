package com.team5430.control;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ControlSystemManager {

        List<ControlSystem> controlSystems;
        private static ControlSystemManager instance;
    
        //**Manage tests for Control Systems with ease! */
        private ControlSystemManager(ControlSystem... controlSystems) {
        //init list
        this.controlSystems = new ArrayList<ControlSystem>();
        }
    
        public static ControlSystemManager getInstance() {
            if (instance == null) {
                instance = new ControlSystemManager();
            }
            return instance;
        }
    
        public ControlSystemManager addControlSystem(ControlSystem... cs){
            if (instance == null) {
                instance = getInstance();
            }else{
                for (ControlSystem c : cs) {
                    controlSystems.add(c);
                }
            }
            return instance;
        }
    
        public static SendableChooser<Boolean> buildTestChooser() {
            SendableChooser<Boolean> controlSystemChooser = new SendableChooser<Boolean>();
            
            var _controlSystems = getInstance().controlSystems;
            
            controlSystemChooser.setDefaultOption("None", false);
    
            //add options to test each control system
            for (ControlSystem cs : _controlSystems ) {
            controlSystemChooser.addOption(cs.getName(), cs.configureTest());
        }

        //assume all tests will pass
        var testAll = true;

        //any failed will cause testAll to be false
        for (ControlSystem cs : _controlSystems) {
            testAll = testAll && cs.configureTest();
        }


        controlSystemChooser.addOption("Test All Control Systems [MAKE SURE ROBOT IS ON BLOCKS]", testAll);

        return controlSystemChooser;
        }

        public void stopAll(){
            for (ControlSystem cs : controlSystems) {
                cs.Stop();
            }
        }   
    
}
