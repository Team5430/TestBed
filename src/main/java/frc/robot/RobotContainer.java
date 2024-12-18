// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import com.team5430.control.CollisionDetection;
import com.team5430.control.ControllerManager;
import com.team5430.control.ControlSystemManager;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.hangSub;
import frc.robot.subsystems.Drive;

public class RobotContainer {

  //dashboard chooser
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> testChooser;
    
      // init subsystems
    
      protected Drive mDrive;
      protected Vision m_Vision;
      protected hangSub m_HangSub;

      protected ControlSystemManager controlSystemManager;

      protected OdometryThread odometryThread;
      
      protected ControllerManager mControllerManager;
      private CollisionDetection collisionFeedback;

      
    
    public RobotContainer() {
    //init  
        //init subsystems
        mDrive = Drive.getInstance();
        m_Vision = Vision.getInstance();
        m_HangSub = hangSub.getInstance();

        controlSystemManager = ControlSystemManager.getInstance().addControlSystem(mDrive, m_HangSub);

        //init feedback
        mControllerManager = ControllerManager.getInstance();
        collisionFeedback = CollisionDetection.getInstance();

        //init odometry thread
        odometryThread = new OdometryThread(mDrive, m_Vision);        
        odometryThread.start();
    
    
        //Pathplanner example to register commands for gui usage
        //NamedCommands.registerCommand("NAME TO REGISTER", new PrintCommand("action"));
    
        //setup autochooser
        autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);

        //setup test chooser
        testChooser = ControlSystemManager.buildTestChooser();
            SmartDashboard.putData("Test Chooser", testChooser);


    //default commands
        // setup drive
          mDrive.setDefaultCommand(
                  new DriveCommand(
                  mControllerManager::getX,
                  mControllerManager::getY,
                  mControllerManager::getRightX,
                  mDrive));
      
    
        configureBindings();
    
     
        
      }
        // controller bindings here
      private void configureBindings() {
    
        // bring hang down
        mControllerManager
            .LeftBumper()
            .onTrue(new hangSub().Down())
            .onFalse(new hangSub().Idle());
            
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

        var TestTab = Shuffleboard.getTab("Tests");
        TestTab.add("Test Control Systems", testChooser);

        //  for use of when alerts ? -> new Trigger(testChooser::getSelected).onTrue(new Alert.set("Test"));
        SmartDashboard.putBoolean("TEST RESULT:", testChooser.getSelected());       
      }

      // stops and resets all control systems
      public void Stop(){
        controlSystemManager.stopAll();
      }

      // get auto command
      public Command getAutonomousCommand(){
        return autoChooser.getSelected();
      }

  
}
