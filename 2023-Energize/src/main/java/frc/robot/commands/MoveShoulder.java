// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveShoulder extends CommandBase {

  private Intake intake;

  private double rotationTarget;

  /** Creates a new MoveShoulder. */
  public MoveShoulder(double rotationTarget, Intake intake) {
    this.intake = intake;
    this.rotationTarget = rotationTarget;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentShoulderPosition = intake.getShoulderPosition();
    if (currentShoulderPosition < rotationTarget) {
      intake.spinShoulder(0.20);
    }
    else if (currentShoulderPosition > rotationTarget) {
      intake.spinShoulder(-0.20);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.spinShoulder(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentShoulderPosition = intake.getShoulderPosition();
    if (Math.abs(currentShoulderPosition - rotationTarget) < 50) {
      return true;
    }
    return false;

  }

}
