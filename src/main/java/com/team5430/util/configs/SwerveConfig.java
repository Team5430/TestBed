package com.team5430.util.configs;

public class SwerveConfig  {
    
    public SwerveConfig(){}
//nest other configs into SwerveConfig
    public SoftwareConfigs Software = new SoftwareConfigs();

    public PhysicalPropertiesConfigs PhysicalProperties = new PhysicalPropertiesConfigs();
}


