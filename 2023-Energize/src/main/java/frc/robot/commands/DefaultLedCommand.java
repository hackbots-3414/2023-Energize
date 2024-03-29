package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Swerve;

public class DefaultLedCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(DefaultLedCommand.class);
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LedSubsystem m_subsystem;
  private double m_color;

  private Intake intake;
  private Swerve swerve;

  public DefaultLedCommand(LedSubsystem subsystem, double color, Intake intake, Swerve swerve) {
    m_subsystem = subsystem;
    m_color = color;
    this.intake = intake;
    this.swerve = swerve;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // DataLogManager.start();
  }

  @Override
  public void execute() {
    if (!swerve.isfieldRelative()) {
      m_subsystem.setColor(-0.09); // Strobe Blue
    } else if (intake.getObjectState()) {
      m_subsystem.setColor(0.75); // Dark Green 
    } else if (DriverStation.isAutonomous()) {
      m_subsystem.setColor(.91); // Purple
    } else if (DriverStation.getMatchTime() < 15) {
      m_subsystem.setColor(-0.05); // Strobe white
    } else if (DriverStation.getMatchTime() <= 30) {
      m_subsystem.setColor(-0.11); // Strobe red
    } else {
      m_subsystem.setColor(m_color);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
