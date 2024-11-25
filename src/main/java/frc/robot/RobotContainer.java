// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team5430.control.CollisionDetection;
import com.team5430.control.ControllerManager;
import com.team5430.util.booleans;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SimDriveCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.hangSub;

public class RobotContainer {

  // init subsystems
  protected Drive m_Drive = new Drive();

  protected Vision m_Vision = new Vision();

  protected hangSub m_HangSub = new hangSub();

  // init controllers
  protected ControllerManager mControllerManager = new ControllerManager();

  // feedback
  CollisionDetection collisionFeedback = new CollisionDetection();

  public RobotContainer() {

    NamedCommands.registerCommand("NAME TO REGISTER", new PrintCommand("action"));

    // setup drive
    if (booleans.getRobot() == booleans.RobotType.REAL_ROBOT) {
      m_Drive.setDefaultCommand(
              new DriveCommand(
              mControllerManager::getX,
              mControllerManager::getY,
              mControllerManager::getRotation,
              mControllerManager::getThrottleSwitch,
              m_Drive));
    } else {
      m_Drive.setDefaultCommand(
              new SimDriveCommand(
              mControllerManager::getX,
              mControllerManager::getY,
              mControllerManager::getRotation,
              m_Drive));
    }

    configureBindings();
      
    
  }
    // controller bindings here
  private void configureBindings() {

    // bring hang down
    mControllerManager
        .LeftBumper()
        .onTrue(new hangSub().Down())
        .onFalse(new hangSub().Stop());

    // Allows zero gyro during run time
    mControllerManager
        .B()
        .onTrue(new InstantCommand(m_Drive::zeroYaw));

    // Auto aim and direct towards april tag in sight
    // NOTE: overrides normal drive control !!! (to be tested)
    mControllerManager
        .A()
        .and(m_Vision.TagInRange())
        .onTrue(
            new DriveCommand(
                m_Vision::proportionalRange,
                mControllerManager::getY,
                m_Vision::proportionalAim,
                mControllerManager::getThrottleSwitch,
                m_Drive));

    // rumble driver whenever there is a hard collision
    collisionFeedback
        .DetectionTrigger()
        .onTrue(new InstantCommand(mControllerManager::setRumbleOn))
        .onFalse(new InstantCommand(mControllerManager::setRumbleOff));

    // use for any object detection when doing camera work?
    // new Trigger(() -> m_Drive.getPose().getX() > 10).onTrue(new PrintCommand("tracking"));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Strafe");
  }
}
