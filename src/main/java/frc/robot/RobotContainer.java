// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team5430.util.ControllerManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.hangSub;

public class RobotContainer {
//init subsystems
  private Drive m_Drive = new Drive();

  private hangSub m_HangSub = new hangSub();

//init controllers
  private ControllerManager mControllerManager = new ControllerManager();

  private double autoDelay = 0;

  public RobotContainer() {

  //dashboard delay
    SmartDashboard.putNumber("Delay", autoDelay);

    // setup drive
    m_Drive.setDefaultCommand(
        new DriveCommand(
            mControllerManager::getX,
            mControllerManager::getY,
            mControllerManager::getRotation,
            mControllerManager::getThrottleSwitch,
            m_Drive));

    configureBindings();
  }

  // contorller bindings here
  private void configureBindings() {

    // bring hang down
    mControllerManager
        .LeftBumper()
        .onTrue(new InstantCommand(m_HangSub::Down))
        .onFalse(new InstantCommand(m_HangSub::Stop));
  }

  public Command getAutonomousCommand() {

//sequence a delay, then drive
    return Commands.sequence(new WaitCommand(autoDelay), m_Drive.DriveToDistance(2));

  }
}
