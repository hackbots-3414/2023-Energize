package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.LedCommand;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final LedSubsystem m_ledSubsystem;

  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {
    m_ledSubsystem = new LedSubsystem();
    configureBindings();
    
  m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41));
  }

  private void configureBindings() {
    JoystickButton aButton = new JoystickButton(m_driverController, 1);
    aButton.whileHeld(new LedCommand(m_ledSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.LedCommand(m_ledSubsystem, .61);
  // }
}
