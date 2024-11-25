package frc.robot.subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team5430.simulation.SimAHRS;
import com.team5430.simulation.SimSwerveModuleGroup;
import com.team5430.swerve.SwerveModuleConstants;
import com.team5430.swerve.SwerveModuleGroup;
import com.team5430.util.TernaryVoid;
import com.team5430.util.booleans;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drive extends SubsystemBase {

//TODO: make it look pretty !!!
    // Swerve Config
    private final SwerveModuleConstants mConfig = new SwerveModuleConstants();

    //swerve logging
    StructArrayPublisher<SwerveModuleState> mPublisher;
    StructPublisher<Pose2d> mPublisher2;

    // Swerve DriveTrain options
    protected SwerveModuleGroup driveTrain;
    protected SimSwerveModuleGroup simDriveTrain;

    // Odometer
    private SwerveDrivePoseEstimator Odometry;

    // Gyro
    public SimAHRS mGyro;

    public Drive() {
        // Initialize based on robot type
        switch (booleans.getRobot()) {
            case REAL_ROBOT:
                mGyro = new SimAHRS(Port.kMXP);
                driveTrain = new SwerveModuleGroup(4, mConfig);
                Odometry = new SwerveDrivePoseEstimator(
                        mConfig.Kinematics, getRotation2d(), driveTrain.getPositions(true), new Pose2d());
                break;
            case SIM_ROBOT:
                //init sim gyro
                mGyro = new SimAHRS();
                mGyro.initSim();

                //create sim drivetrain
                simDriveTrain = new SimSwerveModuleGroup(4, mConfig.Kinematics);
                Odometry = new SwerveDrivePoseEstimator(
                        mConfig.Kinematics, simDriveTrain.getRotation2d(), simDriveTrain.getPositions(true), new Pose2d());
                
                //data publishin
                mPublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct)
                        .publish();
                        
                mPublisher2 = NetworkTableInstance.getDefault()
                        .getStructTopic("/Pose2d", Pose2d.struct)
                        .publish();
                break;
        }

        configurePathPlanner();

        ResetHeading();
    }

    // Drive methods
    public void control(ChassisSpeeds input, Rotation2d robotAngle, boolean isFieldCentric) {
        new TernaryVoid(
            () -> booleans.getRobot() == booleans.RobotType.REAL_ROBOT,
            () -> new TernaryVoid(
                () -> isFieldCentric,
                () -> driveTrain.fieldCentricDrive(input, robotAngle),
                () -> driveTrain.robotRelativeDrive(input)
            ),
            () -> simDriveTrain.robotRelativeDrive(input)
        );
    }

    // Stops the DriveTrain
    public void Stop() {
        new TernaryVoid(
            () -> booleans.getRobot() == booleans.RobotType.REAL_ROBOT,
            () -> driveTrain.Stop(),
            () -> simDriveTrain.stop()
        );
    }

    // Zeros the gyro
    public void ResetHeading() {
        mGyro.reset();
    }

    // Get position
    public Pose2d getPose() {
        return Odometry.getEstimatedPosition();
    }

    // Reset position
    public void resetPose(Pose2d pose) {
        new TernaryVoid(
            () -> booleans.getRobot() == booleans.RobotType.REAL_ROBOT,
            () -> Odometry.resetPosition(getRotation2d(), driveTrain.getPositions(true), pose),
            () -> Odometry.resetPosition(getRotation2d(), simDriveTrain.getPositions(true), pose)
        );
    }

    // Get heading as a Rotation2d
    public Rotation2d getRotation2d() {
        return mGyro != null ? mGyro.getRotation2d() : new Rotation2d(simDriveTrain.getAverageOmegaRadiansPerSecond(true));
    }

    public void zeroYaw() {
        mGyro.zeroYaw();
    }

    // Configure robot control during auton
    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                () -> booleans.getRobot() == booleans.RobotType.REAL_ROBOT ? driveTrain.getCurrentSpeeds() : simDriveTrain.getCurrentSpeeds(),
                speeds -> {
                    new TernaryVoid(
                            Robot::isReal,
                            () -> driveTrain.robotRelativeDrive(speeds),
                            () -> simDriveTrain.robotRelativeDrive(speeds));
                },
                mConfig.pathFollowerConfig,
                booleans.isBlue(),
                this);
    }

    @Override
    public void simulationPeriodic(){

        //update gyro sim
        mGyro.yaw.set(simDriveTrain.getRotation2d().getDegrees());
        
        simDriveTrain.updateSim();
        // Update the simulated drive train positions
        mPublisher.set(simDriveTrain.getStates(true));
        //update the simulated drive train pose2d
        mPublisher2.set(getPose());
        //TODO: autonmous but simulated!!!
        // Update odometry with simulated values
        Odometry.update(getRotation2d(), simDriveTrain.getPositions(true));
        
    }
    // Loops stuff
    @Override
    public void periodic() {
        if (booleans.isAutonomous().getAsBoolean() && Robot.isReal()) {
            Odometry.update(getRotation2d(), driveTrain.getPositions(true));       
        }
    }
}