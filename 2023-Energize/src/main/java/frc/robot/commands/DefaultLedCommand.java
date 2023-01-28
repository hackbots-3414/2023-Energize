package frc.robot.commands;

import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultLedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LedSubsystem m_subsystem;
  private double m_color;

  private boolean done;

  public DefaultLedCommand(LedSubsystem subsystem, double color) {
    m_subsystem = subsystem;
    m_color = color;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    done = false;
  }

  @Override
  public void execute() {
    m_subsystem.setColor(m_color);
    done = true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
