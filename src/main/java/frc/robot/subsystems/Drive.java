package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import com.kauailabs.navx.frc.AHRS;
import com.team5430.control.ControlSystem;
import com.team5430.swerve.Requests;
import com.team5430.swerve.Requests.*;
import com.team5430.swerve.SimModuleIO;
import com.team5430.swerve.SwerveModuleConstants;
import com.team5430.swerve.SwerveModuleGroup;
import com.team5430.swerve.SwerveModuleIO;
import com.team5430.util.booleans;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends ControlSystem {

//TODO: make it look pretty !!!
    // Swerve Config
    private final SwerveModuleConstants mConfig = Constants.SwerveConstants;

    //swerve logging
    StructArrayPublisher<SwerveModuleState> mPublisher;
    StructPublisher<Rotation2d> mGyroPublisher;

    // Swerve DriveTrain options
    protected SwerveModuleGroup driveTrain;


    // Gyro
    public AHRS mGyro;

    private static Drive mInstance;

    public final AtomicReference<Rotation2d> rotation2dRef = new AtomicReference<>(new Rotation2d());

    public static Drive getInstance(){
        if(mInstance == null) mInstance = new Drive();
            return mInstance;
    }

    private Drive() {
//TODO: maybe a system wide robot type in @RobotContainer (???)
        // Initialize based on robot type
        switch (booleans.getRobot()) {
            case REAL_ROBOT:

                //create real drivetrain
                mGyro = new AHRS(Port.kMXP);
                driveTrain = new SwerveModuleGroup(mConfig,
                    new SwerveModuleIO(0, mConfig),
                    new SwerveModuleIO(1, mConfig),
                    new SwerveModuleIO(2, mConfig),
                    new SwerveModuleIO(3, mConfig));    
    
                break;

            case SIM_ROBOT:

                //create sim drivetrain
                driveTrain = new SwerveModuleGroup(mConfig,
                    new SimModuleIO(),
                    new SimModuleIO(),
                    new SimModuleIO(),
                    new SimModuleIO());
                
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
    public void control(Requests request) { 
        driveTrain.control(request);
    }

    //control robot autonmously 
    public void autoControl(ChassisSpeeds speeds) {
        driveTrain.control(new RobotCentricRequest().withSpeeds(speeds));
    }
    
    // Zeros the gyro
    public synchronized void ResetHeading() {
        if (mGyro != null)
        mGyro.reset();
    }
    
//getters
    // Get heading as a Rotation2d
     public synchronized Rotation2d getRotation2d() {
            //check if gyro is connected or not null before using delta data 
    rotation2dRef.set(mGyro != null ? mGyro.getRotation2d() : driveTrain.getRotation2d());
        return rotation2dRef.get();

    }

    //get robot modules positions !!!
    public synchronized SwerveModulePosition[] getModulePositions(){
        return Robot.isReal() 
        ? driveTrain.getPositions(true) : driveTrain.getPositions(true);
    }
    
    //get robot input chassis speeds 
    public ChassisSpeeds getCurrentSpeeds(){
        return Robot.isReal() 
        ? driveTrain.getCurrentSpeeds() : driveTrain.getCurrentSpeeds();
    }

//control system implementation
    //run and rotate slowly
    @Override
    public boolean configureTest(){

    if(RobotState.isTest()){
        try {
                control(new TestRequest());
                DriverStation.reportWarning("Drive Test Succeeded", false);
                return true;
            } catch (Exception e) {
                DriverStation.reportError("Drive Test Failed: " + e.getMessage(), true);
                return false;
            }
        }
        
        return false;
    }

    //check if gyro and drivetrain are connected
    @Override
    public boolean checkStatus() {
        if(mGyro == null || driveTrain == null) return false;
        
        return mGyro.isConnected() && driveTrain.getCurrentSpeeds().equals(getCurrentSpeeds());
    }

    // Stops the DriveTrain
    @Override
    public synchronized void Stop() {
        driveTrain.Stop();
        }


    //sim updating
    @Override 
    public void simulationPeriodic(){
        mPublisher.set(driveTrain.getStates(true));
        mGyroPublisher.set(getRotation2d());
    }
    
}