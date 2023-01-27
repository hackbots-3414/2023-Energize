// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveWrist extends CommandBase {
//Calls variables to be used later in code
  private Intake intake;

  private double rotationTarget;

  /** Creates a new MoveShoulder. */
  public MoveWrist(double rotationTarget, Intake intake) {
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
    // setting the currentWristposition to equal the current shoulder positon in Intake
    double currentWristPosition = intake.getWristPosition();
    // if the currentWristPosition is less than the rotationTarget then we will set the spin speed to 0.20
    if (currentWristPosition < rotationTarget) {
      intake.spinWrist(0.20);
    }
    else if (currentWristPosition > rotationTarget) {
      intake.spinWrist(-0.20);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.spinWrist(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentWristPosition = intake.getWristPosition();
    if (Math.abs(currentWristPosition - rotationTarget) < 50) {
      return true;
    }
    return false;

  }

}
