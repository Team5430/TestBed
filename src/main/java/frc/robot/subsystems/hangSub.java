package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hangSub extends SubsystemBase {

  protected TalonSRX L;
  protected TalonSRX R;

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

  // Stop motors
  public Command Stop() {
    return Commands.runOnce(
        () -> {
          L.set(ControlMode.PercentOutput, 0);
          R.set(ControlMode.PercentOutput, 0);
        },
        this);
  }
}
