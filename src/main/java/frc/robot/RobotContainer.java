// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.team5430.util.CustomXboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.TestBed;

public class RobotContainer {
  // init subsystem
  private TestBed m_TestBed = new TestBed();
  // init new joystick on usb port 0; can be interchanged for any wired controller on Port 0
  public static CommandJoystick driverJoystick = new CommandJoystick(0);

  public RobotContainer() {
    // apply set bindings
    configureBindings();
    // set default command for driver control
 
  }

  

  // contorller bindings here
  private void configureBindings() {

    
    Trigger FIELD_CENTRIC_TOGGLE = new Trigger(driverJoystick.povUp());

    
  
}
  public Command getAutonomousCommand() {
    return Commands.sequence(new InstantCommand(() -> m_TestBed.DriveTrain.DriveToDistance(30), m_TestBed));
  }
}