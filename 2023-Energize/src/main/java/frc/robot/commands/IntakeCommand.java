package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(IntakeCommand.class);

  final Intake intake;
  // private int bounces;
  

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setRunningIntake(true);
    intake.setCurrentLimitOne();
    intake.setObjectStateFalse();
    intake.set(Constants.IntakeConstants.objectHoldSpeedPercent);

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!intake.getObjectState()) {
      intake.set(0.0);
    }
    intake.setCurrentLimitOne();
    intake.setRunningIntake(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
