package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends SubsystemBase {

//TODO: make it look pretty !!!
//TODO: fix Rotation2d in simulation (works but not as intended)
    // Swerve Config
    private final SwerveModuleConstants mConfig = Constants.SwerveConstants;

    //swerve logging
    StructArrayPublisher<SwerveModuleState> mPublisher;

    // Swerve DriveTrain options
    protected SwerveModuleGroup driveTrain;
    protected SimSwerveModuleGroup simDriveTrain;

    //toggle depending on driver perference
    boolean isFieldCentric = true;

    // Gyro
    public SimAHRS mGyro;

    private static Drive mInstance = new Drive(); 

    public static Drive getInstance(){
        return mInstance;

    }

    public Drive() {
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
            () -> simDriveTrain.robotRelativeDrive(input)
        );
    }
    
    //get robot modules positions !!!
    public SwerveModulePosition[] getModulePositions(){
        return Robot.isReal() 
        ? driveTrain.getPositions(true) : simDriveTrain.getPositions(true);
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return Robot.isReal() 
        ? driveTrain.getCurrentSpeeds() : simDriveTrain.getCurrentSpeeds();
    }

    // Stops the DriveTrain
    public void Stop() {
        new TernaryVoid(
            booleans.RobotisReal(),
            () -> driveTrain.Stop(),
            () -> simDriveTrain.stop()
        );
    }

    // Zeros the gyro
    public void ResetHeading() {
        mGyro.reset();
    }

    
    // Get heading as a Rotation2d
    public Rotation2d getRotation2d() {
        return mGyro != null ? mGyro.getRotation2d() : new Rotation2d(simDriveTrain.getAverageOmegaRadiansPerSecond(true));
    }

    @Override
    public void simulationPeriodic(){

        //update gyro sim
        mGyro.yaw.set(simDriveTrain.getRotation2d().getDegrees());
        
        // Update the simulated drive train positions
        simDriveTrain.updateSim();
        mPublisher.set(simDriveTrain.getStates(true));
        
        
    }
   
}