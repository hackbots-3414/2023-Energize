package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(IntakeCommand.class);

  final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setCurrentLimitOne();
    DataLogManager.start();
    intake.set(Constants.IntakeConstants.intakeSpeedPercent);
  }

  @Override
  public void execute() {
    if (!intake.getObjectState()) {
      intake.set(Constants.IntakeConstants.intakeSpeedPercent);
      if (intake.getCurrent() > IntakeConstants.handCurrentThreshold) {
        intake.set(IntakeConstants.intakeSpeedPercent / 5);
        intake.setCurrentLimitTwo();
        intake.setObjectStateTrue();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!intake.getObjectState()) {
      intake.set(0.0);
    }
    intake.setCurrentLimitOne();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
