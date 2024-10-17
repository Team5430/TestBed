
package com.team5430.util;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder CANCoder; 
  private double angleRatio = 1;
  private double driveRatio = 8.14;
  public double currentHeading;
  public double currentThrottle;
  private SwerveModulePosition internalState = new SwerveModulePosition();

  
    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> anglePosition;
    private StatusSignal<Double> angleVelocity;
    private BaseStatusSignal[] signals;

  private double angle_kP = 0.95;
  private double drive_kP = .15;
  private double magEncoderOffset;

  public SwerveModule(){}
  
  public SwerveModule(int AngleMotorCANid, int DriveMotorCANid, int CANCoderCANid, double offset) {
    angleMotor = new TalonFX(AngleMotorCANid);
    driveMotor = new TalonFX(DriveMotorCANid);
     CANCoder = new CANcoder(CANCoderCANid);

     magEncoderOffset = offset;
    motorConfig();
  
  //data as statusSignals
    signals = new BaseStatusSignal[4];
    signals[0] = drivePosition;
    signals[1] = driveVelocity;
    signals[2] = anglePosition;
    signals[3] = angleVelocity;
  }

  public void DriveToDistance(double distance){
    angleMotor.setControl(new PositionDutyCycle(0));
    driveMotor.setControl(new PositionDutyCycle(distance));
  }
  private void motorConfig() {
    // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // gear ratio
    angleConfig.Feedback.SensorToMechanismRatio = angleRatio;
    // proportional gains
    angleConfig.Slot0.kP = angle_kP;
    driveConfig.Slot0.kP = drive_kP;
    
    driveConfig.Feedback.SensorToMechanismRatio = driveRatio;

    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = magEncoderOffset;

    angleConfig.Feedback.FeedbackRemoteSensorID = CANCoder.getDeviceID();
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;



    // apply configurations
    angleMotor.getConfigurator().apply(angleConfig);
    driveMotor.getConfigurator().apply(driveConfig);
    CANCoder.getConfigurator().apply(encoderConfig);

    // zero encoders

    angleMotor.setPosition(magEncoderOffset);
   }

  

  public void setState(SwerveModuleState state){

    var optimize =  SwerveModuleState.optimize(state, internalState.angle);
    //Heading
      double wantedAngle = optimize.angle.getDegrees();
      angleMotor.setControl(new PositionDutyCycle(wantedAngle/360));
      //-Math.abs(((wantedAngle - 270)/360))));
      //Throttle
        double wantedVelocity = optimize.speedMetersPerSecond;
        driveMotor.setControl(new VelocityDutyCycle(wantedVelocity));
  }

    /**
     * SwerveModulePosition is an object which contains the modules position and modules angle
     * @return The current position of the module
     */
    public SwerveModulePosition getPosition(boolean refresh) {
        if(refresh) {
            drivePosition.refresh();
            driveVelocity.refresh();
            anglePosition.refresh();
            angleVelocity.refresh();
        }
        
        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
        double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

        double distance = driveRotations; 
        internalState.distanceMeters = distance;
        Rotation2d angle = Rotation2d.fromRotations(angleRotations);
        internalState.angle = angle;
        
        return internalState;
    }


    public void invertThrottle(boolean input){
      
      driveMotor.setInverted(input);
    }
  public void Stop() { 
    angleMotor.stopMotor();
    driveMotor.stopMotor();
  }

 
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module Telemetry");
    builder.setActuator(true);
    builder.setSafeState(this::Stop);
    builder.addDoubleProperty("Drive Motor Power", null, null);
  }

  
}
