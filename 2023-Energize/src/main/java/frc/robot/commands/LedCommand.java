package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LedCommand extends CommandBase {

  private LedSubsystem m_LedSubsystem;
  final IntakeSubsystem m_Intake; 

  public LedCommand(LedSubsystem m_LedSubsystem, IntakeSubsystem m_Intake) {
    this.m_LedSubsystem = m_LedSubsystem;
    this.m_Intake = m_Intake;
    
    addRequirements(m_LedSubsystem);
  }

  @Override
  public void initialize() {}

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
