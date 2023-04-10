package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class ejectCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(ejectCommand.class);

  final Intake intake;

  public ejectCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.setRunningIntake(false);
    // DataLogManager.start();    
    intake.set(Constants.IntakeConstants.ejectSpeedPercent);
    intake.setCurrentLimitOne();
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
    intake.setObjectStateFalse();
    intake.setCurrentLimitOne();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
