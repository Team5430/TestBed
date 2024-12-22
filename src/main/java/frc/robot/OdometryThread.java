package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team5430.util.booleans;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;

public class OdometryThread implements Runnable {

    private static final int SLEEP_DURATION_MS = 20;

    //init subsystems
    private final Drive mDrive;
    @SuppressWarnings("unused")
    private final Vision mVision;

    private final StructPublisher<Pose2d> mPublisher;
        
    private SwerveDrivePoseEstimator mPoseEstimator;
    public AtomicReference<Pose2d> pose2dReference = new AtomicReference<Pose2d>(getPose2d());

    //thread management
    private final ExecutorService executorService;
    private Future<?> future;

    public OdometryThread(Drive drive, Vision vision) {
        this.executorService = Executors.newSingleThreadExecutor();
        this.mDrive = drive;
        this.mVision = vision;

        //init publisher; publishes robot pose2d
        mPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RobotPose", Pose2d.struct)
        .publish();

        
        //configure path planner
        configurePathPlanner();
    }

 //starts the thread through executor service
public void start() {
    // Submit the thread and store in Future object
    future = executorService.submit(this);
}

//stops the thread
public void stop() {
    // Cancel the task if it is running
    if (future != null && !future.isDone()) {
        future.cancel(true);
    }
    executorService.shutdownNow();
}


    //get pose2d
    public Pose2d getPose2d() {

        return pose2dReference == null ? new Pose2d() : pose2dReference.get();

    }

    //reset pose2d
    public void resetPose2d(Pose2d poseSupplier) {
         mPoseEstimator.resetPosition(mDrive.getRotation2d(), mDrive.getModulePositions(), poseSupplier);
    }
    

    //configure path planner
    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose2d,
                this::resetPose2d,
                mDrive::getCurrentSpeeds,
                mDrive::autoControl,
                Constants.SwerveConstants.autoFollowerConfig,
                booleans.shouldFlip(),
                mDrive);
    }

    @Override
    public void run() {
        DriverStation.reportWarning("Odometry thread started", false);
        //threading settings
        Thread.currentThread().setName("Odometry Thread");

        mPoseEstimator = new SwerveDrivePoseEstimator(
                Constants.SwerveConstants.Kinematics, mDrive.getRotation2d(), mDrive.getModulePositions(), new Pose2d());

                //vision std deviations
        mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, .7));

        //while thread is not interrupted
        while (true) {

            try {

                mPoseEstimator.update(mDrive.getRotation2d(), mDrive.getModulePositions());

                pose2dReference.set(mPoseEstimator.getEstimatedPosition());

                mPublisher.set(pose2dReference.get());
                // TODO: uncomment when simulating vision
                // mPoseEstimator.addVisionMeasurement(mVision.getPose2d(mDrive.getRotation2d().getDegrees()), mVision.getPoseTimestamp());

                //thread runs every 20ms
                Thread.sleep(SLEEP_DURATION_MS);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                DriverStation.reportError("Odometry thread exception: " + e.getMessage(), true);
            }
        }
    }
}