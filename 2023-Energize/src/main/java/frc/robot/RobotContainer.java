package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutonomousFactory;
import frc.robot.autos.DriveStraight;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.GyroBasedBalancing;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LedCommand;
import frc.robot.commands.MoveShoulder;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ejectCommand;
import frc.robot.subsystems.Intake;
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
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton ejectButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final Intake m_Intake = new Intake();

  // Shoulder Movement
  private final JoystickButton testShoulder = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  // Wrist movement
  private final JoystickButton testWrist = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, fieldRelative, openLoop));
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
    intakeButton.whileTrue(new IntakeCommand(m_Intake));
    ejectButton.whileTrue(new ejectCommand(m_Intake));
    testShoulder.whileTrue(new MoveShoulder(Constants.IntakeConstants.shoulderRotationTarget, m_Intake));
    testWrist.whileTrue(new MoveWrist(Constants.IntakeConstants.wristRotationTarget, m_Intake));
  }

  public Command getAutonomousCommand() {
    return new DriveStraight(s_Swerve, 100000000, 0);
  }

  public void resetAngleMotors() {
    s_Swerve.resetAll();
  }
}
