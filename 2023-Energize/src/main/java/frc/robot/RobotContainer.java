package frc.robot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  final static Logger logger = LoggerFactory.getLogger(RobotContainer.class);

  /* Controllers */
  public final Joystick driver = new Joystick(OperatorConstants.kDriverControllerPort);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, 13);
  // private final JoystickButton reducedSpeed = new JoystickButton(driver, 9);
  // private final JoystickButton autoBalance = new JoystickButton(driver,
  // XboxController.Button.kA.value);
  // private final JoystickButton setX = new JoystickButton(driver,
  // XboxController.Button.kX.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, 10); // fix me Swerve subsys overwrites
  // private final JoystickButton ledConeButton = new JoystickButton(driver, 2);
  // private final JoystickButton ledCubeButton = new JoystickButton(driver, 3);
  private final JoystickButton resetModsToAbs = new JoystickButton(driver, 16);

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();

  SendableChooser<Command> pathChooser = new SendableChooser<>();

  public RobotContainer() {

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(1)/0.7,
            () -> -driver.getRawAxis(0)/0.9,
            () -> -driver.getRawAxis(3),
            () -> robotCentric.getAsBoolean())

    );

    configureButtonBindings();

    
    SmartDashboard.putNumber("Time remaining:", DriverStation.getMatchTime());

    if (DriverStation.getMatchTime() < 30) {
      SmartDashboard.putString("Game part:", "ENDGAME");
    } else if (DriverStation.getMatchTime() < 120) {
      SmartDashboard.putString("Game part", "PLAY");
    } else {
      SmartDashboard.putString("Game part", "AUTO");
    }

    SmartDashboard.putData("Reset Wheels", new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    resetModsToAbs.whileTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
  }

  public Command getAutonomousCommand() {
    return pathChooser.getSelected();
  }
}