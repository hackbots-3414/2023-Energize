package frc.robot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutonomousFactory;
import frc.robot.autos.AutonomousFactory.AutonChoice;
import frc.robot.autos.AutonomousFactory.Bays;
import frc.robot.autos.AutonomousFactory.Heights;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveShoulder;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.PIDBalance;
import frc.robot.commands.Rotate;
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
  private final JoystickButton ejectButton = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton stowAndLowButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton midButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton highButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton pickUpButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

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

  SendableChooser<AutonChoice> pathChooser = new SendableChooser<>();
  SendableChooser<Bays> bayChooser = new SendableChooser<>();
  SendableChooser<Heights> heightChooser = new SendableChooser<>();

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

    m_ledSubsystem.setDefaultCommand(new DefaultLedCommand(m_ledSubsystem, .41));

    configureButtonBindings();

    autons = AutonomousFactory.getInstance(s_Swerve, m_Intake, m_Wrist, m_Shoulder);

    pathChooser.setDefaultOption("Drive Out Bottom", AutonChoice.TestingPath);
    // pathChooser.addOption("Drive Out Top", AutonChoice.DriveOutTop);
    // pathChooser.addOption("Bottom Object One", AutonChoice.BottomObjectOne);
    // pathChooser.addOption("Bottom Object Two", AutonChoice.BottomObjectTwo);
    // pathChooser.addOption("Bottom Object Three", AutonChoice.BottomObjectThree);
    // pathChooser.addOption("Bottom Object Four", AutonChoice.BottomObjectFour);
    // pathChooser.addOption("Bottom Object Five", AutonChoice.BottomObjectFive);
    // pathChooser.addOption("Bottom Object Six", AutonChoice.BottomObjectSix);
    // pathChooser.addOption("Bottom Object Seven", AutonChoice.BottomObjectSeven);
    // pathChooser.addOption("Bottom Object Eight", AutonChoice.BottomObjectEight);
    // pathChooser.addOption("Top Object One", AutonChoice.TopObjectOne);
    // pathChooser.addOption("Top Object Two", AutonChoice.TopObjectTwo);
    // pathChooser.addOption("Top Object Three", AutonChoice.TopObjectThree);
    // pathChooser.addOption("Top Object Four", AutonChoice.TopObjectFour);
    // pathChooser.addOption("Top Object Five", AutonChoice.TopObjectFive);
    // pathChooser.addOption("Top Object Six", AutonChoice.TopObjectSix);
    // pathChooser.addOption("Top Object Seven", AutonChoice.TopObjectSeven);
    // pathChooser.addOption("Top Object Eight", AutonChoice.TopObjectEight);
    // pathChooser.addOption("Auto Balance", AutonChoice.AutoBalance);
    // pathChooser.addOption("Top Start", AutonChoice.TopStart);
    // pathChooser.addOption("Mid Top Start", AutonChoice.MidStartTop);
    // pathChooser.addOption("Mid Low Start", AutonChoice.MidStartLow);
    // pathChooser.addOption("Low Start", AutonChoice.LowStart);

    // bayChooser.setDefaultOption("Low Low", Bays.LowLow);
    // bayChooser.addOption("Low Mid", Bays.LowLow);
    // bayChooser.addOption("Low High", Bays.LowLow);
    // bayChooser.addOption("Mid Low", Bays.MidLow);
    // bayChooser.addOption("Mid Mid", Bays.MidLow);
    // bayChooser.addOption("Mid High", Bays.MidLow);
    // bayChooser.addOption("High Low", Bays.HighLow);
    // bayChooser.addOption("High Mid", Bays.HighLow);
    // bayChooser.addOption("High High", Bays.HighLow);

    // heightChooser.setDefaultOption("Low", Heights.Low);
    // heightChooser.addOption("Mid", Heights.Mid);
    // heightChooser.addOption("High", Heights.High);
    
    SmartDashboard.putData("Auton Mode", pathChooser);

    SmartDashboard.putNumber("Time remaining:", DriverStation.getMatchTime());

    if (DriverStation.getMatchTime() < 30){
      SmartDashboard.putString("Game part:", "ENDGAME");
    } else if (DriverStation.getMatchTime() < 120){
      SmartDashboard.putString("Game part","PLAY");
    } else {
      SmartDashboard.putString("Game part", "AUTO");
    }

  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  private void configureButtonBindings() {

    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    setX.whileTrue(new InstantCommand(() -> s_Swerve.setX()));
    autoBalance.whileTrue(new PIDBalance(s_Swerve, true));
    //SmartDashboard.putData(new Rotate(s_Swerve));
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
    return autons.eventChooser(pathChooser.getSelected()/*, bayChooser.getSelected(), heightChooser.getSelected()*/);
  }
}