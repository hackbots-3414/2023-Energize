package frc.robot.commands; 


import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class DefaultLedCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(DefaultLedCommand.class);
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
    logger.info("logging");
    DataLogManager.start();
    done = false;
  }

  @Override
  public void execute() {
    if (DriverStation.isAutonomous()){
      m_subsystem.setColor(.91);
    } else if (DriverStation.getMatchTime()<15){
      m_subsystem.setColor(.65);
    } else if (DriverStation.getMatchTime()<=30){
      m_subsystem.setColor(-.25);
    }
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
