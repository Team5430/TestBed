package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import com.team5430.control.ControlSystem;
import com.team5430.simulation.SimAHRS;
import com.team5430.simulation.SimSwerveModuleGroup;
import com.team5430.swerve.SwerveModuleConstants;
import com.team5430.swerve.SwerveModuleGroup;
import com.team5430.util.TernaryVoid;
import com.team5430.util.booleans;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends ControlSystem {

//TODO: make it look pretty !!!
//TODO: fix Rotation2d in simulation (works but not as intended)
    // Swerve Config
    private final SwerveModuleConstants mConfig = Constants.SwerveConstants;

    //swerve logging
    StructArrayPublisher<SwerveModuleState> mPublisher;
    StructPublisher<Rotation2d> mGyroPublisher;

    // Swerve DriveTrain options
    protected SwerveModuleGroup driveTrain;
    protected SimSwerveModuleGroup simDriveTrain;

    //toggle depending on driver perference
    boolean isFieldCentric = true;

    // Gyro
    public SimAHRS mGyro;

    private static Drive mInstance = new Drive(); 

    public final AtomicReference<Rotation2d> rotation2dRef = new AtomicReference<>(new Rotation2d());

    public static Drive getInstance(){
        return mInstance;
    }

    private Drive() {

        // Initialize based on robot type
        switch (booleans.getRobot()) {
            case REAL_ROBOT:
                mGyro = new SimAHRS(Port.kMXP);
                driveTrain = new SwerveModuleGroup(4, mConfig);
    
                break;

            case SIM_ROBOT:
                //init sim gyro
                mGyro = new SimAHRS();
                mGyro.initSim();

                //create sim drivetrain
                simDriveTrain = new SimSwerveModuleGroup(4, mConfig.Kinematics);
                
                //data publishing
                mPublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct)
                        .publish();

                mGyroPublisher = NetworkTableInstance.getDefault()
                        .getStructTopic("/Gyro", Rotation2d.struct)
                        .publish();        

                break;
            default:
                DriverStation.reportError("Invalid Robot Type", true);
                break;
            

            }

        ResetHeading();
    }

    // Drive methods
    public void control(ChassisSpeeds input) {
        new TernaryVoid(
            booleans.RobotisReal(),
            () -> new TernaryVoid(
                () -> isFieldCentric,
                () -> driveTrain.fieldCentricDrive(input, getRotation2d()),
                () -> driveTrain.robotRelativeDrive(input)
            ),
            () -> simDriveTrain.fieldCentricDrive(input, getRotation2d())
        );
    }
    

    // Zeros the gyro
    public synchronized void ResetHeading() {
        mGyro.reset();
    }

    // Get heading as a Rotation2d
       public synchronized Rotation2d getRotation2d() {

        var rotation2d = mGyro != null ? mGyro.getRotation2d() : new Rotation2d(simDriveTrain.getRotation2d().getDegrees());
            rotation2dRef.set(rotation2d);
        return rotation2dRef.get();

    }

    //get robot modules positions !!!
    public SwerveModulePosition[] getModulePositions(){
        return Robot.isReal() 
        ? driveTrain.getPositions(true) : simDriveTrain.getPositions(true);
    }
    
    //get robot input chassis speeds 
    public ChassisSpeeds getCurrentSpeeds(){
        return Robot.isReal() 
        ? driveTrain.getCurrentSpeeds() : simDriveTrain.getCurrentSpeeds();
    }

    
    @Override
    //run and rotate slowly
    public boolean configureTest(){
        try {
            control(new ChassisSpeeds(.1, .1, 1));
            DriverStation.reportWarning("Drive Test Succeeded", false);
            return true;
        } catch (Exception e) {
            DriverStation.reportError("Drive Test Failed: " + e.getMessage(), true);
            return false;
        }
    }

    @Override
    public boolean checkStatus() {
        if(mGyro == null || driveTrain == null) return false;
        
        return mGyro.isConnected() && driveTrain.getCurrentSpeeds().equals(getCurrentSpeeds());
    }

    // Stops the DriveTrain
    @Override
    public synchronized void Stop() {
        new TernaryVoid(
            booleans.RobotisReal(),
                () -> driveTrain.Stop(),
                () -> simDriveTrain.stop()
            );
        }
   
    //sim updating
    @Override 
    public void simulationPeriodic(){
        simDriveTrain.updateSim();        
        mGyro.yaw.set(simDriveTrain.getYaw());
        mPublisher.set(simDriveTrain.getStates(true));
        mGyroPublisher.set(mGyro.getRotation2d());
    }
}