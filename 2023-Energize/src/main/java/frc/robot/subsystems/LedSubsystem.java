package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  
  Spark ledcontroller;

  public LedSubsystem() {
    ledcontroller = new Spark(9);
    ledcontroller.set(0.4);
  }

  public void setColor(double color) {
    ledcontroller.set(color);
  }
}
