package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ejectCommand extends CommandBase {

  final Intake m_Intake;

  public ejectCommand(Intake intake) {
    m_Intake = intake;
  }

  @Override
  public void initialize() {
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
