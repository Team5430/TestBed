// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team5430.util.CustomXboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TestBed;

public class RobotContainer {
  // init subsystem
  private TestBed m_TestBed = new TestBed();
  // init new joystick on usb port 0; can be interchanged for any wired controller on Port 0
  public static CustomXboxController driverJoystick = new CustomXboxController(0);

  public RobotContainer() {
    // apply set bindings
    configureBindings();
    // set default command for driver control
    m_TestBed.setDefaultCommand(
        new RunCommand(
            () ->
                m_TestBed.drive(
                    driverJoystick.getLeftX(),
                    driverJoystick.getLeftY(),
                    driverJoystick.getRightX(),
                    driverJoystick.getRightTriggerAxis()),
            m_TestBed));
  }

  // contorller bindings here
  private void configureBindings() {

    /*
    Trigger controllerVibration = new Trigger(m_TestBed.mfeedback::CollisionDetected);

    controllerVibration.onTrue(new InstantCommand(() -> driverJoystick.setRumble(true)));
    controllerVibration.onFalse(new InstantCommand(() -> driverJoystick.setRumble(false)));
  */
}
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}