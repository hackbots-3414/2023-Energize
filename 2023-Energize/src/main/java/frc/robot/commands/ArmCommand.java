// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import ch.qos.logback.classic.Logger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.IntakeAngles;
import frc.robot.subsystems.Shoulder;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private Wrist wrist;
  private Shoulder shoulder;
  private int selector;
  
  public ArmCommand(Shoulder shoulder, Wrist wrist, int selector) {
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.selector = selector;

    addRequirements(shoulder);
    addRequirements(wrist);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (selector) {

      // stowed
      case 0:
        //wrist.motionMagic(IntakeAngles.stowedWristAngle);
        shoulder.motionMagic(IntakeAngles.stowedShoulderAngle);
        break;

      // pick up
      case 1:
        //wrist.motionMagic(IntakeAngles.pickUpWristAngle);
        shoulder.motionMagic(IntakeAngles.pickUpShoulderAngle);
        break;

      // low
      case 2:
        //wrist.motionMagic(IntakeAngles.lowWristAngle);
        shoulder.motionMagic(IntakeAngles.lowShoulderAngle);
        break;

      // mid
      case 3:
        //wrist.motionMagic(IntakeAngles.midWristAngle);
        shoulder.motionMagic(IntakeAngles.midShoulderAngle);
        break;

      // high
      case 4:
        //wrist.motionMagic(IntakeAngles.highWristAngle);
        shoulder.motionMagic(IntakeAngles.highShoulderAngle);
        break;
      
      // shelf
      case 5:
        //wrist.motionMagic(IntakeAngles.shelfWristAngle);
        shoulder.motionMagic(IntakeAngles.shelfShoulderAngle);
        break;
    }
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
