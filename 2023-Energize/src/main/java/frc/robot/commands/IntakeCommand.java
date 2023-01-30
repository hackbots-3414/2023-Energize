package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DataLogManager;



public class IntakeCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(IntakeCommand.class);

  final Intake m_Intake;

  public IntakeCommand(Intake intake) {
      m_Intake = intake;
  }

  @Override
  public void initialize() {
    DataLogManager.start();
    m_Intake.spinHand(Constants.IntakeConstants.intakeSpeedPercent);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Intake.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
