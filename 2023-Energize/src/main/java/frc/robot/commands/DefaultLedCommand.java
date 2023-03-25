package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;

public class DefaultLedCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(DefaultLedCommand.class);
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LedSubsystem m_subsystem;
  private double m_color;

  private boolean done;

  private Intake intake;

  public DefaultLedCommand(LedSubsystem subsystem, double color, Intake intake) {
    m_subsystem = subsystem;
    m_color = color;
    this.intake = intake;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // logger.info("logging");
    DataLogManager.start();
    done = false;
  }

  @Override
  public void execute() {

    if (intake.getObjectState()) {
      m_subsystem.setColor(.75); // Dark Green
    } else if (DriverStation.isAutonomous()) {
      m_subsystem.setColor(.91); // Purple
    } else if (DriverStation.getMatchTime() < 15) {
      m_subsystem.setColor(-0.05); // Strobe white
    } else if (DriverStation.getMatchTime() <= 30) {
      m_subsystem.setColor(-.25); //heart beat red
    } else {
      m_subsystem.setColor(m_color);
    }
    done = true;

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
