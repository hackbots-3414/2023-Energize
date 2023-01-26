package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutonomousFactory;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.GyroBasedBalancing;
import frc.robot.commands.LedCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  /* Controllers */
  private final Joystick driver = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.kOperatorControllerPort);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton balance = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton setX = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Operator Buttons */
  public final JoystickButton aButton = new JoystickButton(operator, XboxController.Button.kA.value);
  public final JoystickButton xButton = new JoystickButton(operator, XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();

  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, fieldRelative, openLoop));
    m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41));

    configureBindings();
  }

  private void configureBindings() {
    JoystickButton aButton = new JoystickButton(driver, 1);
    aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    JoystickButton xButton = new JoystickButton(driver, 3);
    xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));


    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    balance.onTrue(new GyroBasedBalancing(s_Swerve));
    setX.whileTrue(new InstantCommand(() -> s_Swerve.setX()));

    /* Operator Buttons */
    aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
  }

  public Command getAutonomousCommand() {
    return AutonomousFactory.getInstance(s_Swerve).testAuto();
  }

  public void resetAngleMotors() {
    s_Swerve.resetAll();
  }
}
