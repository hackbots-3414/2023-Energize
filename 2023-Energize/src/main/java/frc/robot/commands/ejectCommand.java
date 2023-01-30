package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DataLogManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class ejectCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(ejectCommand.class);


  final Intake m_Intake;

  public ejectCommand(Intake intake) {
    m_Intake = intake;
  }

  @Override
  public void initialize() {
    DataLogManager.start();
    m_Intake.spinHand(Constants.IntakeConstants.ejectSpeedPercent);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Intake.stopHand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
