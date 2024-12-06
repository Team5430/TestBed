package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team5430.util.ControlSystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hangSub extends SubsystemBase implements ControlSystem {

  protected TalonSRX L;
  protected TalonSRX R;


  protected static hangSub mInstance = new hangSub();

  public static hangSub getInstance(){
    return mInstance;
  }

  public hangSub() {

    // init motors
    L = new TalonSRX(10);
    R = new TalonSRX(11);
    // invert motor
    R.setInverted(true);
  }

  // hang only goes down mechanically
  public Command Down() {
    return Commands.runOnce(
        () -> {
          L.set(ControlMode.PercentOutput, -.5);
          R.set(ControlMode.PercentOutput, -.5);
        },
        this);
  }

  @Override
  //pretty much just unwind the string
  public void configureTest(){
    L.set(ControlMode.PercentOutput, .1);
    R.set(ControlMode.PercentOutput, .1);
  }

  // Stop motors
  @Override
  public void Stop() {
    L.set(ControlMode.PercentOutput, 0);
    R.set(ControlMode.PercentOutput, 0);
  }




  
}
