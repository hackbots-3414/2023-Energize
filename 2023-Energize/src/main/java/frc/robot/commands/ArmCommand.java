// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import ch.qos.logback.classic.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;
import frc.robot.Constants.IntakeAngles;
import frc.robot.subsystems.Shoulder;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private Wrist wrist;
  private Shoulder shoulder;
  private int selector;
  private boolean isTeleop;

  public ArmCommand(Shoulder shoulder, Wrist wrist, int selector, boolean isTeleop) {
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.selector = selector;
    this.isTeleop = isTeleop;

    addRequirements(shoulder);
    addRequirements(wrist);
  }

  public ArmCommand(Shoulder shoulder, Wrist wrist, int selector) {
    this(shoulder, wrist, selector, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (selector) {

      // stowed & Low
      case 0:
        wrist.setGoal(Constants.IntakeAngles.stowedWristAngle);
        //Timer.delay(0.1);
        shoulder.setGoal(Constants.IntakeAngles.stowedShoulderAngle);
        break;

      // pick up
      case 1:
        shoulder.setGoal(Constants.IntakeAngles.pickUpShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.pickUpWristAngle);
        break;

      // mid
      case 3:
        shoulder.setGoal(Constants.IntakeAngles.midShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.midWristAngle);
        break;

      // high
      case 4:
        shoulder.setGoal(Constants.IntakeAngles.highShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.highWristAngle);
        break;

      // shelf
      case 5:
        shoulder.setGoal(Constants.IntakeAngles.shelfShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.shelfWristAngle);
        break;

      // standing cone
      case 6:
      shoulder.setGoal(Constants.IntakeAngles.standingConeShoulderAngle);
      wrist.setGoal(Constants.IntakeAngles.standingConeWristAngle);
      break;
    }
    shoulder.enable();
    wrist.enable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!isTeleop) {
      Timer.delay(1.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isTeleop) {
      return false;
    }
    return true;
  }
}
