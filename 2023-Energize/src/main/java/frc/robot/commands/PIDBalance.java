// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDBalance extends PIDCommand {
  private boolean alwaysRun;

  /** Creates a new PIDBalance. */
  public PIDBalance(Swerve swerve, boolean alwaysRun) {
    super(
        // The controller that the command will use
        new PIDController(Constants.BalanceConstants.KP, Constants.BalanceConstants.KI, Constants.BalanceConstants.KD),
        // This should return the measurement
        () -> {return swerve.getPitch().getDegrees();},
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {if (output > .5){
          output = .05;
        } else if (output < -.5){
          output = -.05;
        }
        //SmartDashboard.putNumber("output", output);
        swerve.drive(new Translation2d(output, 0.0), 0, false, true);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1.8, .1);
    SmartDashboard.putData(getController());
    this.alwaysRun = alwaysRun;
    
    addRequirements(swerve);
  }

  public PIDBalance(Swerve swerve) {
    this(swerve, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !alwaysRun || getController().atSetpoint();
  }
}
