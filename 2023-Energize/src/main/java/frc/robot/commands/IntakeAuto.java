// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeAutoConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;;

public class IntakeAuto extends CommandBase {
  /** Creates a new IntakeAuto. */
  private Intake intake;
  private Wrist wrist;
  private Shoulder shoulder;
  private int selector;

  public IntakeAuto(Wrist wrist, Shoulder shoulder, int selector) {
    this.wrist = wrist;
    this.shoulder = shoulder;
    this.selector = selector;

    addRequirements(wrist);
    addRequirements(shoulder);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (selector) {
      case 0:
        wrist.moveWrist(IntakeAutoConstants.lowWristAngle);
        shoulder.moveShoulder(IntakeAutoConstants.lowShoulderAngle);
        break;

      case 1:
        wrist.moveWrist(IntakeAutoConstants.midWristAngle);
        shoulder.moveShoulder(IntakeAutoConstants.midShoulderAngle);
        break;

      case 2:
        wrist.moveWrist(IntakeAutoConstants.highWristAngle);
        shoulder.moveShoulder(IntakeAutoConstants.highShoulderAngle);
        break;
      
      case 3:
        wrist.moveWrist(IntakeAutoConstants.shelfWristAngle);
        shoulder.moveShoulder(IntakeAutoConstants.shelfShoulderAngle);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}