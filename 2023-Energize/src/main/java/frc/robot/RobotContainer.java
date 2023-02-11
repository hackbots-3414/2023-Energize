package frc.robot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.GyroBasedBalancing;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;


public class RobotContainer {

  final static Logger logger = LoggerFactory.getLogger(RobotContainer.class);

  /* Controllers */
  private final Joystick driver = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.kOperatorControllerPort);


  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton balance = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton setX = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


  /* Operator Buttons */
  public final JoystickButton aButton = new JoystickButton(operator, XboxController.Button.kA.value);
  public final JoystickButton xButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton ejectButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shoulder m_Shoulder = new Shoulder();
  private final Wrist m_Wrist = new Wrist();

  // Shoulder Movement
  private final JoystickButton testShoulder = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  // Wrist movement
  private final JoystickButton testWrist = new JoystickButton(driver, XboxController.Button.kRightBumper.value);


  private boolean openLoop = false;


  public RobotContainer() {

    s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -OI.getLeftVertical(), 
                () -> -OI.getLeftLateral(), 
                () -> -OI.getRightLateral(), 
                () -> robotCentric.getAsBoolean()
            )

        );

    m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41));


    configureBindings();
  }

  private void configureBindings() {
    // JoystickButton aButton = new JoystickButton(driver, 1);
    // aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    // JoystickButton xButton = new JoystickButton(driver, 3);
    // xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));

    JoystickButton tempA = new JoystickButton(driver, 1);
    tempA.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.defaultArmAngle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.defaultWristAngle))));
    JoystickButton tempB = new JoystickButton(driver, 2);
    tempB.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.mediumArmAngle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.mediumWristAngle))));
    JoystickButton tempX = new JoystickButton(driver, 3);
    tempX.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.lowArmangle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.lowWristAngle))));
    JoystickButton tempY = new JoystickButton(driver, 4);    
    tempY.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.highArmAngle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.highWristAngle))));

    JoystickButton rightBumper = new JoystickButton(driver, 6);
    rightBumper.whileTrue(new InstantCommand(() -> m_Shoulder.moveShoulderDown(-Constants.IntakeConstants.speed, Constants.IntakeConstants.shoulderLowerLimit)));

    

    JoystickButton leftBumper = new JoystickButton(driver, 5);
    leftBumper.whileTrue(new InstantCommand(() -> m_Shoulder.moveShoulderDown(Constants.IntakeConstants.speed, Constants.IntakeConstants.shoulderLowerLimit)));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    balance.onTrue(new GyroBasedBalancing(s_Swerve));
    setX.whileTrue(new InstantCommand(() -> s_Swerve.setX()));

    /* Operator Buttons */
    // aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    // xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    // intakeButton.whileTrue(new IntakeCommand(m_Intake, m_Shoulder, m_Wrist));
    // ejectButton.whileTrue(new ejectCommand(m_Intake));
    // testShoulder.whileTrue(new MoveShoulder(Constants.IntakeConstants.shoulderRotationTarget, m_Intake));
    // testWrist.whileTrue(new MoveWrist(Constants.IntakeConstants.wristRotationTarget, m_Intake));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    // return AutonomousFactory.getInstance(s_Swerve).testAuto();
    // return new DriveStraight(s_Swerve, 1, 0);
    return new exampleAuto(s_Swerve);
  }
}
