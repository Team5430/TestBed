package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hangSub extends SubsystemBase {

  private TalonSRX L;
  private TalonSRX R;

  public hangSub() {

//init motors
    L = new TalonSRX(10);
    R = new TalonSRX(11);
//invert motor
    R.setInverted(true);

  }

//hang only goes down mechanically
  public void Down() {
    L.set(ControlMode.PercentOutput, -.5);
    R.set(ControlMode.PercentOutput, -.5);
  }

//Stop motors 
  public void Stop() {
    L.set(ControlMode.PercentOutput, 0);
    R.set(ControlMode.PercentOutput, 0);
  }
}
