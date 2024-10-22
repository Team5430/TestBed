package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.team5430.util.MathHelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestBed;

public class DriveCommand extends Command {
    
    TestBed mTestBed;

    DoubleSupplier
     xTranslation,
     yTranslation,
     rTranslation,
     ThrottleBreaker;

    Boolean FIELD_CENTRIC = false;
    

    public DriveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier Rotation, DoubleSupplier breaking, TestBed subsystem){
        xTranslation = X;
        yTranslation = Y;
        rTranslation = Rotation;
        ThrottleBreaker = breaking;
        mTestBed = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){} 

    @Override
    public void execute(){
        double x = xTranslation.getAsDouble();
        double y = yTranslation.getAsDouble();
        double rotation = rTranslation.getAsDouble();
        double breaking = MathHelpers.VariableSpeedDecline(ThrottleBreaker.getAsDouble());

        boolean DriveStyleToggle = FIELD_CENTRIC;
        Rotation2d RobotAngle = mTestBed.mGyro.getRotation2d();

        ChassisSpeeds Inputs = new ChassisSpeeds(
                      x * breaking,
                      y * breaking,
                     rotation);

    mTestBed.Drive(Inputs, RobotAngle, DriveStyleToggle);
    }

    @Override
    public void end(boolean interupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
