// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class IntakeAutomaticCommand extends IntakeCommand {

  Swerve swerve;

  /** Creates a new IntakeAutomaticCommand. */
  public IntakeAutomaticCommand(Swerve swerve, Intake intake) {
    super(intake);
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute(); // Spin the motor!
    // We assume that the arm is already raised.
    boolean tooClose = intake.getIRState();
    if (!tooClose) {
      swerve.driveForward(Constants.IntakeAutomatic.fullSpeed, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerve.driveForward(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIRState();
  }
}
