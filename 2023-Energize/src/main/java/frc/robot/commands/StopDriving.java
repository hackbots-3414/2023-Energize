// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class StopDriving extends CommandBase {
  Swerve s_Swerve;
  Intake m_Intake;
  
  public StopDriving(Swerve s_Swerve, Intake m_Intake) {
    addRequirements(s_Swerve);
    this.s_Swerve = s_Swerve;
    this.m_Intake = m_Intake;
  }

  @Override
  public void initialize() {
    s_Swerve.drive(new Translation2d(), 0, true, true);
  }

  @Override
  public boolean isFinished() {
    return m_Intake.getObjectState();
  }
}
