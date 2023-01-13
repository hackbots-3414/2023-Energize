// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveStraight extends CommandBase {
  private final Swerve swerve;
  private final double distance;
  private final double direction;
  private final double heading;
  private static double distanceGone;

  private final Translation2d myTranslation2d;


  public DriveStraight(Swerve swerve, double distance, double direction, double heading) {
    this.swerve = swerve;
    this.distance = distance;
    this.direction = direction;
    this.heading = heading;
    
    myTranslation2d = new Translation2d(distance, direction);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(myTranslation2d, heading, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     swerve.drive(myTranslation2d, 0.0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    distanceGone = swerve.getPose().getX();
    if (distanceGone > distance) {
      return true;
    } else {
      return false;
    }
  }
}
