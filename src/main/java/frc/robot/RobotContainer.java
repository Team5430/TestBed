// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team5430.util.CollisionDetection;
import com.team5430.util.ControllerManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.hangSub;

public class RobotContainer {

  //dashboard menu
  private final SendableChooser<Command> autoChooser;

  // init subsystems
  protected Drive m_Drive = new Drive();

  protected hangSub m_HangSub = new hangSub();

  // init controllers
  protected ControllerManager mControllerManager = new ControllerManager();

  // feedback
  CollisionDetection collisionFeedback = new CollisionDetection();

  public RobotContainer() {

    //init autoChooser
    autoChooser = AutoBuilder.buildAutoChooser();

    //put menu on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

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

  // controller bindings here
  private void configureBindings() {

    // bring hang down
    mControllerManager
        .LeftBumper()
        .onTrue(new InstantCommand(m_HangSub::Down))
        .onFalse(new InstantCommand(m_HangSub::Stop));

    // Allows zero gyro during run time
    mControllerManager.B().onTrue(new InstantCommand(m_Drive.mGyro::zeroYaw));

    // rumble driver whenever there is a hard collision
    collisionFeedback
        .DetectionTrigger()
        .onTrue(new InstantCommand(mControllerManager::setRumbleOn))
        .onFalse(new InstantCommand(mControllerManager::setRumbleOff));

    // use for any object detection when doing camera work?
    // new Trigger(() -> m_Drive.getPose().getX() > 10).onTrue(new PrintCommand("tracking"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
