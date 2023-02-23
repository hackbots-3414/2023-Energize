package frc.robot;

import java.util.ArrayList;
import java.util.Collection;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutonomousFactory;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.GyroBasedBalancing;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveShoulder;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ejectCommand;
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
  private final JoystickButton zeroGyro = new JoystickButton(driver, 13);
  private final JoystickButton autoBalance = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton setX = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Operator Buttons */
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton ejectButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton stowAndLowButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton midButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton highButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton pickUpButton = new JoystickButton(operator, XboxController.Button.kB.value);

  private final POVButton shoulderUp = new POVButton(operator, 90);
  private final POVButton shoulderDown = new POVButton(operator, 270);
  private final POVButton wristUp = new POVButton(operator, 0);
  private final POVButton wristDown = new POVButton(operator, 180);


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shoulder m_Shoulder = new Shoulder();
  private final Wrist m_Wrist = new Wrist();

  private boolean openLoop = false;

  SendableChooser<Command> autonChooser = new SendableChooser<>();


  public RobotContainer() {

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(1),
            () -> driver.getRawAxis(0),
            () -> driver.getRawAxis(3),
            () -> robotCentric.getAsBoolean())

    );

    m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41));

    configureBindings();

    ShuffleboardTab tab = Shuffleboard.getTab("AutonChoosers");
    ArrayList<NetworkTableEntry> networkTables = new ArrayList<>();

    SmartDashboard.putBoolean("Top 1", false);
    SmartDashboard.putBoolean("Top 2", false);
    SmartDashboard.putBoolean("Top 3", false);
    SmartDashboard.putBoolean("Top 4", false);
    SmartDashboard.putBoolean("Top 5", false);
    SmartDashboard.putBoolean("Top 6", false);
    SmartDashboard.putBoolean("Top 7", false);
    SmartDashboard.putBoolean("Top 8", false);
    SmartDashboard.putBoolean("Top 9", false);
    SmartDashboard.putBoolean("Mid 1", false);
    SmartDashboard.putBoolean("Mid 2", false);
    SmartDashboard.putBoolean("Mid 3", false);
    SmartDashboard.putBoolean("Mid 4", false);
    SmartDashboard.putBoolean("Mid 5", false);
    SmartDashboard.putBoolean("Mid 6", false);
    SmartDashboard.putBoolean("Mid 7", false);
    SmartDashboard.putBoolean("Mid 8", false);
    SmartDashboard.putBoolean("Mid 9", false);
    SmartDashboard.putBoolean("Low 1", false);
    SmartDashboard.putBoolean("Low 2", false);
    SmartDashboard.putBoolean("Low 3", false);
    SmartDashboard.putBoolean("Low 4", false);
    SmartDashboard.putBoolean("Low 5", false);
    SmartDashboard.putBoolean("Low 6", false);
    SmartDashboard.putBoolean("Low 7", false);
    SmartDashboard.putBoolean("Low 8", false);
    SmartDashboard.putBoolean("Low 9", false);

    ArrayList<Boolean> inputs = new ArrayList<>();
    
    inputs.add(SmartDashboard.getBoolean("Top 1", false));
    inputs.add(SmartDashboard.getBoolean("Top 2", false));
    inputs.add(SmartDashboard.getBoolean("Top 3", false));
    inputs.add(SmartDashboard.getBoolean("Top 4", false));
    inputs.add(SmartDashboard.getBoolean("Top 5", false));
    inputs.add(SmartDashboard.getBoolean("Top 6", false));
    inputs.add(SmartDashboard.getBoolean("Top 7", false));
    inputs.add(SmartDashboard.getBoolean("Top 8", false));
    inputs.add(SmartDashboard.getBoolean("Top 9", false));
    inputs.add(SmartDashboard.getBoolean("Mid 1", false));
    inputs.add(SmartDashboard.getBoolean("Mid 2", false));
    inputs.add(SmartDashboard.getBoolean("Mid 3", false));
    inputs.add(SmartDashboard.getBoolean("Mid 4", false));
    inputs.add(SmartDashboard.getBoolean("Mid 5", false));
    inputs.add(SmartDashboard.getBoolean("Mid 6", false));
    inputs.add(SmartDashboard.getBoolean("Mid 7", false));
    inputs.add(SmartDashboard.getBoolean("Mid 8", false));
    inputs.add(SmartDashboard.getBoolean("Mid 9", false));
    inputs.add(SmartDashboard.getBoolean("Low 1", false));
    inputs.add(SmartDashboard.getBoolean("Low 2", false));
    inputs.add(SmartDashboard.getBoolean("Low 3", false));
    inputs.add(SmartDashboard.getBoolean("Low 4", false));
    inputs.add(SmartDashboard.getBoolean("Low 5", false));
    inputs.add(SmartDashboard.getBoolean("Low 6", false));
    inputs.add(SmartDashboard.getBoolean("Low 7", false));
    inputs.add(SmartDashboard.getBoolean("Low 8", false));
    inputs.add(SmartDashboard.getBoolean("Low 9", false));

    autonChooser.setDefaultOption("Test Path", AutonomousFactory.getInstance(s_Swerve, m_Intake, m_Wrist, m_Shoulder, inputs).testEvents());
    SmartDashboard.putNumber("Time remaining:", DriverStation.getMatchTime());

    if (DriverStation.getMatchTime() < 15){
      SmartDashboard.putString("Game part:", "ENDGAME");
    } else if (DriverStation.getMatchTime() < 135){
      SmartDashboard.putString("Game part","PLAY");
    } else {
      SmartDashboard.putString("Game part", "AUTO");
    }
  }

  public Swerve getSwerve() {
    return s_Swerve;
  }


  private void configureBindings() {
    // JoystickButton aButton = new JoystickButton(driver, 1);
    // aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    // JoystickButton xButton = new JoystickButton(driver, 3);
    // xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));

    // JoystickButton tempA = new JoystickButton(driver, 1);
    // tempA.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.defaultArmAngle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.defaultWristAngle))));
    // JoystickButton tempB = new JoystickButton(driver, 2);
    // tempB.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.mediumArmAngle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.mediumWristAngle))));
    // JoystickButton tempX = new JoystickButton(driver, 3);
    // tempX.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.lowArmangle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.lowWristAngle))));
    // JoystickButton tempY = new JoystickButton(driver, 4);    
    // tempY.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> m_Shoulder.moveShoulder(Constants.IntakeConstants.highArmAngle)), new InstantCommand(() -> m_Wrist.moveWrist(Constants.IntakeConstants.highWristAngle))));

    // JoystickButton rightBumper = new JoystickButton(driver, 6);
    // rightBumper.whileTrue(new InstantCommand(() -> m_Shoulder.moveShoulderDown(-Constants.IntakeConstants.speed, Constants.IntakeConstants.shoulderLowerLimit)));

    

    // JoystickButton leftBumper = new JoystickButton(driver, 5);
    // leftBumper.whileTrue(new InstantCommand(() -> m_Shoulder.moveShoulderDown(Constants.IntakeConstants.speed, Constants.IntakeConstants.shoulderLowerLimit)));
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putNumber("Time remaining:", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Time remaining:", DriverStation.getMatchTime());

    if (DriverStation.getMatchTime() < 15) {
      SmartDashboard.putString("Game part:", "ENDGAME");
    } else if (DriverStation.getMatchTime() < 135) {
      SmartDashboard.putString("Game part", "PLAY");
    } else {
      SmartDashboard.putString("Game part", "AUTO");
    }
  }

  private void configureButtonBindings() {

    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    setX.whileTrue(new InstantCommand(() -> s_Swerve.setX()));
    autoBalance.whileTrue(new PIDBalance(s_Swerve, true));

    /* Operator Buttons */
    // aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    // xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    intakeButton.whileTrue(new IntakeCommand(m_Intake));
    ejectButton.whileTrue(new ejectCommand(m_Intake));
    stowAndLowButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 0));
    midButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 3));
    highButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 4));
    pickUpButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 1));

    shoulderUp.whileTrue(new MoveShoulder(m_Shoulder, Constants.IntakeConstants.shoulderMoveSpeedPercentage));
    shoulderDown.whileTrue(new MoveShoulder(m_Shoulder, -Constants.IntakeConstants.shoulderMoveSpeedPercentage));
    wristUp.whileTrue(new MoveWrist(m_Wrist, Constants.IntakeConstants.wristMoveSpeedPercentage));
    wristDown.whileTrue(new MoveWrist(m_Wrist, -Constants.IntakeConstants.wristMoveSpeedPercentage));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}