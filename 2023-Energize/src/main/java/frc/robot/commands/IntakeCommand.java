package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
    intake.setRunningIntake(true);
    intake.setCurrentLimitOne();
    intake.setObjectStateFalse();
    intake.setCurrentLimitOne();
    intake.set(Constants.IntakeConstants.intakeSpeedPercent);
  }

  @Override
  public void execute() {
    if (intake.getCurrent() > IntakeConstants.handCurrentThreshold) {
      intake.setCurrentLimitTwo();
      intake.setObjectStateTrue();
      intake.set(IntakeConstants.objectHoldSpeedPercent);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) { // only runs if button was let go without object
      intake.set(0.0);
      intake.setRunningIntake(false);
      intake.setObjectStateFalse();
    }
  }

  @Override
  public boolean isFinished() {
    return intake.getObjectState();
  }
}
