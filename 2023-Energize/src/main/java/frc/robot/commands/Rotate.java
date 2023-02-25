// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Rotate extends CommandBase {
  /** Creates a new Rotate. */
  final Swerve swerve;
  public double startingGyro;
  public Rotate(Swerve s_swerve) {
    this.swerve = s_swerve;
    addRequirements(s_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.zeroGyro();
    startingGyro = swerve.getYaw().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new Translation2d(), .5, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(swerve.getYaw().getDegrees()-startingGyro) > 179){
      return true;
    } 
    return false;
  }
}
