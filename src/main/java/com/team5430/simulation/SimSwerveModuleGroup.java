package com.team5430.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimSwerveModuleGroup {

    private SimSwerveModule[] simSwerveModules = new SimSwerveModule[4];
    private int moduleCount;
    private SwerveDriveKinematics kinematics;

// stats
    private Rotation2d robotAngle;
    private SwerveModulePosition[] deltaPositions;


    //pretty much the same thing as the real one
    /**
     * Initialize the simulation-based swerve module group.
     * @param kinematics Swerve drive kinematics used for conversions
     * @param moduleCount Number of swerve modules to initialize (max 4)
     */
    public SimSwerveModuleGroup(int moduleCount, SwerveDriveKinematics kinematics) {
        this.moduleCount = moduleCount;
        this.kinematics = kinematics;

        this.robotAngle = Rotation2d.fromDegrees(0);

        this.deltaPositions = new SwerveModulePosition[moduleCount];

        for (int i = 0; i < moduleCount; i++) {
            simSwerveModules[i] = new SimSwerveModule();
            deltaPositions[i] = new SwerveModulePosition(0, new Rotation2d(0));
        }
     
    }

    /**
     * Set the states of all swerve modules.
     * @param states Desired swerve module states
     */
    public void setStates(SwerveModuleState... states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0); // Example max velocity
        for (int i = 0; i < moduleCount; i++) {
            simSwerveModules[i].updateStateAndPosition(states[i]);
        }
    }

    /**
     * Drive the robot relative to itself.
     * @param speeds Chassis speeds to drive the robot
     */
    public void robotRelativeDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        setStates(states);
    }

    /**
     * Drive the robot relative to the field using the given robot angle.
     * @param speeds Chassis speeds
     * @param robotAngle Robot's current angle
     */
    public void fieldCentricDrive(ChassisSpeeds speeds, Rotation2d robotAngle) {
        robotRelativeDrive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle));
    }

    /**
     * Get the current speeds of the robot.
     * @return Current chassis speeds
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getStates(true));
    }

    /**
     * Get the positions of all swerve modules.
     * @param refresh Whether to refresh the state from simulation
     * @return Array of current swerve module positions
     */
    public SwerveModulePosition[] getPositions(boolean refresh) {
        SwerveModulePosition[] positions = new SwerveModulePosition[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            positions[i] = simSwerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * Get the states of all swerve modules.
     * @param refresh Whether to refresh the state from simulation
     * @return Array of current swerve module states
     */
    public SwerveModuleState[] getStates(boolean refresh) {
        SwerveModuleState[] states = new SwerveModuleState[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            states[i] = simSwerveModules[i].getState();
        }
        return states;
    }

    /*
     * Get the robot's current angle as a simulated Rotation2d.
     * 
     * @return Robot's current angle
     */
    public Rotation2d getRotation2d() {
     
        for(int i = 0; i < moduleCount; i++){
            deltaPositions[i] = simSwerveModules[i].getModuleDelta();
        }

        var twist = kinematics.toTwist2d(deltaPositions);
    
        robotAngle = robotAngle.plus(new Rotation2d(twist.dtheta));
        return robotAngle;
        

    }


}