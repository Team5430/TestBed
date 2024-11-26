package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team5430.util.booleans;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class OdometryThread extends Thread{
    
    
    
    //init subsystems
    protected Drive mDrive = Drive.getInstance();

    protected Vision mVision = Vision.getInstance();

    //init pose estimator
    protected SwerveDrivePoseEstimator mPoseEstimator;

    //init for data reading !!!
    StructPublisher<Pose2d> mPublisher;

    
    public OdometryThread(){                

        super();

        configurePathPlanner();

        mPublisher = NetworkTableInstance.getDefault()
                     .getStructTopic("/RobotPose", Pose2d.struct)
                     .publish();
    }

    //get estimated robot pose on field
    public Pose2d getPose2d(){
        if (mPoseEstimator != null) {
            return mPoseEstimator.getEstimatedPosition();
        } else {
            return new Pose2d();
        }
    }

    //reset robot pose if needed
    public void resetPose2d(Pose2d poseSupplier){
        if (mPoseEstimator != null) {
            mPoseEstimator.resetPosition(mDrive.getRotation2d(), mDrive.getModulePositions(), poseSupplier);
        }    
    }

    private void configurePathPlanner(){
        AutoBuilder.configureHolonomic(
            this::getPose2d,
            this::resetPose2d,
            mDrive::getCurrentSpeeds, 
            mDrive::control, 
            Constants.SwerveConstants.pathFollowerConfig,
            booleans.isBlue(),
            mDrive);
    }

    @Override
    public void run(){
        Threads.setCurrentThreadPriority(true, 1);

        mPoseEstimator = new SwerveDrivePoseEstimator
                    (Constants.SwerveConstants.Kinematics, mDrive.getRotation2d(), mDrive.getModulePositions(), new Pose2d());

        mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, .7));


        
        //while not interupted
        while(!Thread.currentThread().isInterrupted()){

            try {
                mPublisher.accept(getPose2d());
                mPoseEstimator.update(mDrive.getRotation2d(), mDrive.getModulePositions());

                // TODO: uncomment when simulating vision !!!
                // mPoseEstimator.addVisionMeasurement(mVision.getPose2d(mDrive.getRotation2d().getDegrees()), mVision.getPoseTimestamp());

                // Adjust sleep time for required update rate
                Thread.sleep(20); // for example, 20 milliseconds update rate

            } catch (InterruptedException e) {
                // Handle thread interruption (e.g., when stopping the thread)
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                // Report Error 
                DriverStation.reportError("Odometry Thread:" + e.getMessage(), e.getStackTrace());
            }
        }
        
    }
}
