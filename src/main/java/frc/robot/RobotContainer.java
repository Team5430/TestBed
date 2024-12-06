// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team5430.control.CollisionDetection;
import com.team5430.control.ControlSystem;
import com.team5430.control.ControllerManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.hangSub;
import frc.robot.subsystems.Drive;

public class RobotContainer {

  //auton dashboard chooser
    private final SendableChooser<Command> autoChooser;

  // init subsystems
  List<ControlSystem> controlSystems = List.of(Drive.getInstance(), hangSub.getInstance());

  protected Drive mDrive = Drive.getInstance();

  protected Vision m_Vision = Vision.getInstance();

  protected hangSub m_HangSub = hangSub.getInstance();

  //init odometry thread
  protected OdometryThread odometryThread = new OdometryThread(mDrive, m_Vision);

  // init controllers
  protected ControllerManager mControllerManager = new ControllerManager();

  // feedback
  CollisionDetection collisionFeedback = new CollisionDetection();


  public RobotContainer() {

    //Pathplanner example to register commands for gui usage
    //NamedCommands.registerCommand("NAME TO REGISTER", new PrintCommand("action"));

    //setup autochooser
    autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    // setup drive
      mDrive.setDefaultCommand(
              new DriveCommand(
              mControllerManager::getX,
              mControllerManager::getY,
              mControllerManager::getRightX,
              mDrive));
  

    configureBindings();

    //setup odometry thread
    odometryThread.start();
 
    
  }
    // controller bindings here
  private void configureBindings() {

    // bring hang down
    mControllerManager
        .LeftBumper()
        .onTrue(new hangSub().Down())
        .onFalse(new InstantCommand(m_HangSub::Stop));
        
    // Auto aim and direct towards april tag in sight
    //TODO: test -> NOTE: overrides normal drive control !!!)
    mControllerManager
        .A()
        .and(m_Vision.TagInRange())
        .onTrue(
            new DriveCommand(
                m_Vision::proportionalRange,
                mControllerManager::getY,
                m_Vision::proportionalAim,
                mDrive));

    // rumble driver whenever there is a hard collision
    collisionFeedback
        .DetectionTrigger()
        .onTrue(new InstantCommand(mControllerManager::setRumbleOn))
        .onFalse(new InstantCommand(mControllerManager::setRumbleOff));

    // use for any object detection when doing camera work?
    // new Trigger(() -> m_Drive.getPose().getX() > 10).onTrue(new PrintCommand("tracking"));
  }

  // configure tests for each control system
  public void configureTests(){

    //run tests for each control system
    for (ControlSystem controlSystem : controlSystems) {
      SmartDashboard.putBoolean(controlSystem.getClass().getSimpleName(), controlSystem.configureTest());
    }

  }

  // stop all control systems
  public void Stop(){
    mDrive.Stop();
    m_HangSub.Stop();
  }

  // get auto command
  public Command getAutonomousCommand() {

    try{
      //load auto
      return autoChooser.getSelected();

    }catch (Exception e){

      DriverStation.reportError("AUTO FAILED:" + e.getMessage(),  e.getStackTrace());
      return Commands.none();

    }

  }
}
