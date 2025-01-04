package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team5430.control.ControlSystem;
import com.team5430.util.ConstantsParser;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class hangSub extends ControlSystem {

//init motors
  protected TalonSRX L;
  protected TalonSRX R;
 
  // Enum for the state of the hang
  private enum HangState {
    IDLE(0),
    DOWN(-1);

   private double power;

    HangState(double power) {
      this.power = power;
    }

  }

  private static ConstantsParser.LocalConstants constants;

  // Singleton instance
  protected static hangSub mInstance = new hangSub();

  public static hangSub getInstance(){
    return mInstance;
  }
  
  public hangSub() {

    // Parse constants
    try {

      constants = new ConstantsParser(new File(Filesystem.getDeployDirectory(), "constants/hangproperties.json")).getLocalConstants();

    } catch (Exception e){
      // If there is an error, throw a runtime exception
      throw new RuntimeException(e);

    }
    // init motors
    L = new TalonSRX(constants.getCANids()[0]);
    R = new TalonSRX(constants.getCANids()[1]);
    
    // invert motor
    R.setInverted(true);

  }

//set power to the hang
  private Command setPower(HangState state) {
    return Commands.runOnce(
        () -> {
          L.set(ControlMode.PercentOutput, state.power);
          R.set(ControlMode.PercentOutput, state.power);
        },
        this);
  }

  // Set the state of the hang
  public Command Down() { return setPower(HangState.DOWN);}

  public Command Idle() { return setPower(HangState.IDLE);}


  @Override
  //pretty much just unwind the string
  public boolean configureTest(){
    try {
      setPower(HangState.DOWN).execute();
      new WaitCommand(.5);
      setPower(HangState.IDLE).execute();
      DriverStation.reportWarning("HangSub Test Passed", false);
      return true;
    } catch (Exception e) {
      DriverStation.reportError("HangSub Test Failed:" + e.getMessage(), true);
      return false;
    }
        
  
  }

  // Stop motors
  @Override
  public void Stop() {
    Idle().execute();
  }

  //any problems will appear with the motors
  @Override
  public boolean checkStatus() {
    return true;
  }




  
}
