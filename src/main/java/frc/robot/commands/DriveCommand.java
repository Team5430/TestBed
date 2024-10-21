package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.MathUtil;
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

    boolean FIELD_CENTRIC;
    

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
        double breaking = ThrottleBreaker.getAsDouble();

        ChassisSpeeds Inputs = new ChassisSpeeds(
                      MathUtil.applyDeadband(x, .3) * breaking,
                      MathUtil.applyDeadband(y, .3) * breaking,
                     MathUtil.applyDeadband(rotation, .3));

    if(FIELD_CENTRIC){
        mTestBed.request(
            new ChassisSpeeds(x, y, rotation)
            );
        }else{
        mTestBed.request(
            ChassisSpeeds.fromFieldRelativeSpeeds(Inputs, null)
           );
        }
    }

    @Override
    public void end(boolean interupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
