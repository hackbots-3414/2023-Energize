package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  private double time;
  private double startTime;

  public Wait(double seconds) {
    this.time = seconds;
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime)/1000 >= time;
  }
}
