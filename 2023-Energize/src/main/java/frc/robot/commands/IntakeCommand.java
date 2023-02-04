package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DataLogManager;



public class IntakeCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(IntakeCommand.class);

  final Intake m_Intake;
  final Shoulder m_Shoulder;
  final Wrist m_Wrist;

  public IntakeCommand(Intake intake, Shoulder shoulder, Wrist wrist) {
      m_Intake = intake;
      m_Shoulder = shoulder;
      m_Wrist = wrist;
  }

  @Override
  public void initialize() {
    DataLogManager.start();
    m_Intake.spinHand(Constants.IntakeConstants.intakeSpeedPercent);
    m_Wrist.setSpeed(Constants.IntakeConstants.intakeSpeedPercent);
    m_Shoulder.spin(Constants.IntakeConstants.intakeSpeedPercent);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Shoulder.stop();
    m_Wrist.stopWrist();
    m_Intake.stopHand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
