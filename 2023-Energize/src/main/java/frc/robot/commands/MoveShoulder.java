package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class MoveShoulder extends CommandBase {

  private Shoulder shoulder;
  private double speed;

  public MoveShoulder(Shoulder shoulder, double speed) {
    this.shoulder = shoulder;
    this.speed = speed;
    addRequirements(shoulder);
  }

  @Override
  public void initialize() {
    shoulder.disable();
  }

  @Override
  public void execute() {
    shoulder.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    shoulder.set(0.0);
    shoulder.setGoal(shoulder.getMeasurement());
    shoulder.enable();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
