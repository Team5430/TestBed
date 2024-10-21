// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.team5430.util.ControllerManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.TestBed;

public class RobotContainer {
  // init subsystem
  private TestBed m_TestBed = new TestBed();

  private ControllerManager mControllerManager;
  
  public RobotContainer() {
  

  //setup drive
    m_TestBed.setDefaultCommand
    (new DriveCommand(
      mControllerManager::getX,
       mControllerManager::getY,
       mControllerManager::getRotation,
      mControllerManager::getThrottleSwitch,
      mControllerManager.quickTrigger(),
       m_TestBed));

    configureBindings();
 
  }

  // contorller bindings here
  private void configureBindings() {

  //exmaple usage

    mControllerManager.A().onTrue(new PrintCommand("A was pressed"));

    
}
  public Command getAutonomousCommand() {
    return Commands.sequence(m_TestBed.DriveToDistance(2));
  }
}