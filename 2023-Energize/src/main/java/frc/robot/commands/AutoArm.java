// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class AutoArm extends CommandBase {
  private Wrist wrist;
  private Shoulder shoulder;
  private int selector;

  public AutoArm(Shoulder shoulder, Wrist wrist, int selector) {
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.selector = selector;

    addRequirements(shoulder);
    addRequirements(wrist);
  }

  @Override
  public void execute() {
    switch (selector) {

      // Stow
      case 0:
        wrist.setGoal(Constants.IntakeAngles.stowedWristAngle);
        // Timer.delay(0.1);
        shoulder.setGoal(Constants.IntakeAngles.stowedShoulderAngle);
        break;

      // Floor Pickup
      case 1:
        shoulder.setGoal(Constants.IntakeAngles.pickUpShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.pickUpWristAngle);
        break;

      // Mid
      case 3:
        shoulder.setGoal(Constants.IntakeAngles.midShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.midWristAngle);
        break;

      // High
      case 4:
        shoulder.setGoal(Constants.IntakeAngles.highShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.highWristAngle);
        break;

      // Shelf
      case 5:
        shoulder.setGoal(Constants.IntakeAngles.shelfShoulderAngle);
        wrist.setGoal(Constants.IntakeAngles.shelfWristAngle);
        break;

      // Standing Cone
      case 6:
        wrist.setGoal(Constants.IntakeAngles.standingConeWristAngle);
        shoulder.setGoal(Constants.IntakeAngles.standingConeShoulderAngle);
        break;

        // Shelf Cone
      case 7:
        shoulder.setGoal(Constants.IntakeAngles.shelfShoulderDown);
        wrist.setGoal(Constants.IntakeAngles.shelfWristAngle);
        break;
    }
    shoulder.enable();
    wrist.enable();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
