// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class IntakeAutomaticCommand extends IntakeCommand {

  final static Logger logger = LoggerFactory.getLogger(IntakeAutomaticCommand.class);

  Swerve swerve;

  /** Creates a new IntakeAutomaticCommand. */
  public IntakeAutomaticCommand(Swerve swerve, Intake intake) {
    super(intake);
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    logger.debug("Hello and good tidings from IntakeAutomaticCommand!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute(); // Spin the motor!
    // We assume that the arm is already raised.
    boolean notTooClose = super.intake.getIRState();
    logger.debug("IR Sensor returned: tooClose={}", notTooClose);
    if (notTooClose) {
      swerve.driveForward(Constants.IntakeAutomatic.shelfApproachSpeed, 0);
    } else {
      swerve.driveForward(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerve.driveForward(0,0);
    logger.debug("Finishing IntakeAutomaticCommand!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
