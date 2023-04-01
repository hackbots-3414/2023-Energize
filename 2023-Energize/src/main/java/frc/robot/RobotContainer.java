package frc.robot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutonomousFactory;
import frc.robot.autos.AutonomousFactory.AutonChoice;
import frc.robot.autos.AutonomousFactory.Bays;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoArm;
import frc.robot.commands.DefaultLedCommand;
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
  public final Joystick driver = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.kOperatorControllerPort);
  private final Joystick buttonBoard = new Joystick(Constants.ButtonBoard.buttonBoardPort);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, 13);
  // private final JoystickButton reducedSpeed = new JoystickButton(driver, 9);
  // private final JoystickButton autoBalance = new JoystickButton(driver, XboxController.Button.kA.value);
  // private final JoystickButton setX = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, 10); //fix me Swerve subsys overwrites
  // private final JoystickButton ledConeButton = new JoystickButton(driver, 2);
  // private final JoystickButton ledCubeButton = new JoystickButton(driver, 3);

  /* Operator Buttons */
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton ejectButton = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton stowAndLowButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton midButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton highButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton pickUpButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton shelfButton = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton standingConeButton = new JoystickButton(operator, XboxController.Button.kBack.value);

  private final POVButton shoulderUp = new POVButton(operator, 90);
  private final POVButton shoulderDown = new POVButton(operator, 270);
  private final POVButton wristUp = new POVButton(operator, 0);
  private final POVButton wristDown = new POVButton(operator, 180);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shoulder m_Shoulder = new Shoulder();
  private final Wrist m_Wrist = new Wrist(m_Shoulder);

  /* Buttonboard things: */
  private final JoystickButton button0 = new JoystickButton(buttonBoard, 1);
  private final JoystickButton button1 = new JoystickButton(buttonBoard, 2);
  private final JoystickButton button2 = new JoystickButton(buttonBoard, 3);
  private final JoystickButton button3 = new JoystickButton(buttonBoard, 4);
  private final JoystickButton button4 = new JoystickButton(buttonBoard, 5);
  private final JoystickButton button5 = new JoystickButton(buttonBoard, 6);
  private final JoystickButton button6 = new JoystickButton(buttonBoard, 7);
  private final JoystickButton button7 = new JoystickButton(buttonBoard, 8);
  private final JoystickButton button8 = new JoystickButton(buttonBoard, 9);
  private final JoystickButton button9 = new JoystickButton(buttonBoard, 10);
  private final JoystickButton button10 = new JoystickButton(buttonBoard, 11);
  private final JoystickButton button11 = new JoystickButton(buttonBoard, 12);


  SendableChooser<Command> pathChooser = new SendableChooser<>();

  private AutonomousFactory autons;

  public RobotContainer() {

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(1),
            () -> -driver.getRawAxis(0),
            () -> -driver.getRawAxis(3),
            () -> robotCentric.getAsBoolean())

    );

    m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41, m_Intake, s_Swerve));

    configureButtonBindings();

    autons = AutonomousFactory.getInstance(s_Swerve, m_Intake, m_Wrist, m_Shoulder);

    SmartDashboard.putData("Auton Mode", pathChooser);

    // pathChooser.setDefaultOption("Drive Out Bottom", AutonChoice.Balance);
    pathChooser.setDefaultOption("Nothing", autons.eventChooser(AutonChoice.Nothing));
    pathChooser.addOption("Wall", autons.eventChooser(AutonChoice.Left));
    pathChooser.addOption("Barrier", autons.eventChooser(AutonChoice.Right)); 
    pathChooser.addOption("Balance", autons.eventChooser(AutonChoice.Balance)); 
    pathChooser.addOption("Wall High", autons.eventChooser(AutonChoice.WallHigh));
    pathChooser.addOption("Barrier High", autons.eventChooser(AutonChoice.BarrierHigh));
    pathChooser.addOption("Barrier High Two Object", autons.eventChooser(AutonChoice.BarrierHighTwoObject));
    pathChooser.addOption("Balance High", autons.eventChooser(AutonChoice.BalanceHigh));
    pathChooser.addOption("Test", autons.eventChooser(AutonChoice.Test));

    // Button board configurations:
    button3.whileTrue(new ProxyCommand(autons::bayChooser1));
    button4.whileTrue(new ProxyCommand(autons::bayChooser2));
    button5.whileTrue(new ProxyCommand(autons::bayChooser3));
    button6.whileTrue(new ProxyCommand(autons::bayChooser4));
    button7.whileTrue(new ProxyCommand(autons::bayChooser5));
    button8.whileTrue(new ProxyCommand(autons::bayChooser6));
    button9.whileTrue(new ProxyCommand(autons::bayChooser7));
    button10.whileTrue(new ProxyCommand(autons::bayChooser8));
    button11.whileTrue(new ProxyCommand(autons::bayChooser9));

    SmartDashboard.putNumber("Time remaining:", DriverStation.getMatchTime());

    if (DriverStation.getMatchTime() < 30){
      SmartDashboard.putString("Game part:", "ENDGAME");
    } else if (DriverStation.getMatchTime() < 120){
      SmartDashboard.putString("Game part","PLAY");
    } else {
      SmartDashboard.putString("Game part", "AUTO");
    }

    SmartDashboard.putData("Coast Mode", new InstantCommand(() -> armCoastMode()));
    SmartDashboard.putData("Brake Mode", new InstantCommand(() -> armBrakeMode()));

  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  public Intake getIntake() {
    return m_Intake;
  }

  private void configureButtonBindings() {

    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    // setX.whileTrue(new InstantCommand(() -> s_Swerve.setX()));
    // autoBalance.whileTrue(new PIDBalance(s_Swerve, true));
    //SmartDashboard.putData(new Rotate(s_Swerve));
    /* Operator Buttons */
    // aButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    // xButton.whileTrue(new LedCommand(m_ledSubsystem, m_Intake));
    intakeButton.whileTrue(new IntakeCommand(m_Intake));
    ejectButton.whileTrue(new ejectCommand(m_Intake));
    //stowAndLowButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 0));
    stowAndLowButton.onTrue(
      Commands.runOnce(
        () -> {
        m_Shoulder.setGoal(Constants.IntakeAngles.stowedShoulderAngle);
        m_Wrist.setGoal(Constants.IntakeAngles.stowedWristAngle);
        m_Wrist.enable();
        m_Shoulder.enable();
    },
    m_Shoulder, m_Wrist));

    shelfButton.onTrue(
      Commands.runOnce(
      () -> {
        m_Shoulder.setGoal(Constants.IntakeAngles.shelfShoulderAngle);
        m_Wrist.setGoal(Constants.IntakeAngles.shelfWristAngle);
        m_Wrist.enable();
        m_Shoulder.enable();
    },
    m_Shoulder, m_Wrist));
    
    midButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 3));
    highButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 4));
    pickUpButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 1));
    // shelfButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 5));
    standingConeButton.whileTrue(new ArmCommand(m_Shoulder, m_Wrist, 6));

    shoulderUp.whileTrue(new MoveShoulder(m_Shoulder, Constants.IntakeConstants.shoulderMoveSpeedPercentage));
    shoulderDown.whileTrue(new MoveShoulder(m_Shoulder, -Constants.IntakeConstants.shoulderMoveSpeedPercentage));
    wristUp.whileTrue(new MoveWrist(m_Wrist, Constants.IntakeConstants.wristMoveSpeedPercentage));
    wristDown.whileTrue(new MoveWrist(m_Wrist, -Constants.IntakeConstants.wristMoveSpeedPercentage));

    

    button0.whileTrue(new AutoArm(m_Shoulder, m_Wrist, 4));
    button1.whileTrue(new AutoArm(m_Shoulder, m_Wrist, 3));
    button2.whileTrue(new AutoArm(m_Shoulder, m_Wrist, 0));
    
  }


  public Command getAutonomousCommand() {
    return pathChooser.getSelected();
  }

  public void armBrakeMode() {
    m_Wrist.setBrakeMode();
    m_Shoulder.setBrakeMode();
  }

  public void armCoastMode() {
    m_Wrist.setCoastMode();
    m_Shoulder.setCoastMode();
  }
}