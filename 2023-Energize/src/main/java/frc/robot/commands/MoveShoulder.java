package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveShoulder extends CommandBase {

  private Intake intake;

  private double rotationTarget;

  public MoveShoulder(double rotationTarget, Intake intake) {
    this.intake = intake;
    this.rotationTarget = rotationTarget;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double currentShoulderPosition = intake.getShoulderPosition();
    if (currentShoulderPosition < rotationTarget) {
      intake.spinShoulder(0.20);
    }
    else if (currentShoulderPosition > rotationTarget) {
      intake.spinShoulder(-0.20);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.spinShoulder(0);
  }

  @Override
  public boolean isFinished() {
    double currentShoulderPosition = intake.getShoulderPosition();
    if (Math.abs(currentShoulderPosition - rotationTarget) < 50) {
      return true;
    }
    return false;
  }

}
