package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Intake;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DataLogManager;



public class LedCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(LedCommand.class);


  private LedSubsystem m_LedSubsystem;
  final Intake m_Intake; 

  public LedCommand(LedSubsystem m_LedSubsystem, Intake m_Intake) {
    this.m_LedSubsystem = m_LedSubsystem;
    this.m_Intake = m_Intake;
    
    addRequirements(m_LedSubsystem);
  }

  @Override
  public void initialize() {
    DataLogManager.start();
  }

  @Override
  public void execute() {
    m_LedSubsystem.setColor(0.53);

    if (m_Intake.getIRInput()) {
      m_LedSubsystem.setColor(0.55);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
