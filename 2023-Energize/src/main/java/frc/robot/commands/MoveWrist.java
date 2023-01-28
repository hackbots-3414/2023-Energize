package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveWrist extends CommandBase {
  private Intake intake;

  private double rotationTarget;

  public MoveWrist(double rotationTarget, Intake intake) {
    this.intake = intake;
    this.rotationTarget = rotationTarget;

    addRequirements(intake);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double currentWristPosition = intake.getWristPosition();
    if (currentWristPosition < rotationTarget) {
      intake.spinWrist(0.20);
    }
    else if (currentWristPosition > rotationTarget) {
      intake.spinWrist(-0.20);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.spinWrist(0);
    
  }

  @Override
  public boolean isFinished() {
    double currentWristPosition = intake.getWristPosition();
    if (Math.abs(currentWristPosition - rotationTarget) < 50) {
      return true;
    }
    return false;
  }

}
