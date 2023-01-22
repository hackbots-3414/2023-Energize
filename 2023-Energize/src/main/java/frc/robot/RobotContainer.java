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
  private final LedSubsystem m_ledSubsystem;
  private final IntakeSubsystem m_Intake;

  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  /* Controllers */
  private final Joystick driver = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.kOperatorControllerPort);

  /* Driver Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Operator Controls */
  public final

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton balance = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton setX = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  public RobotContainer() {
    m_ledSubsystem = new LedSubsystem();
    m_Intake = new IntakeSubsystem();

    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41));

    configureBindings();
  }

  private void configureBindings() {
    JoystickButton aButton = new JoystickButton(m_driverController, 1);
    aButton.whileHeld(new LedCommand(m_ledSubsystem, m_Intake));
    JoystickButton xButton = new JoystickButton(m_driverController, 3);
    xButton.whileHeld(new LedCommand(m_ledSubsystem, m_Intake));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    balance.onTrue(new GyroBasedBalancing(s_Swerve));
    setX.whileTrue(new InstantCommand(() -> s_Swerve.setX()));
  }

  public Command getAutonomousCommand() {
    return AutonomousFactory.getInstance(s_Swerve).testAuto();
  }

  public void resetAngleMotors() {
    s_Swerve.resetAll();
  }
}
