// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class StopDriving extends CommandBase {
  Swerve s_Swerve;
  Intake m_Intake;
  
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  
  public StopDriving(Swerve s_Swerve, Intake m_Intake, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    addRequirements(s_Swerve);
    this.s_Swerve = s_Swerve;
    this.m_Intake = m_Intake;
    
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    
    rotationVal *= Constants.IntakeConstants.slowTurn;
    
    translationVal *= Constants.IntakeAutomatic.shelfApproachSpeed;
    strafeVal *= Constants.IntakeAutomatic.shelfApproachSpeed;

    s_Swerve.drive(
      new Translation2d(
        translationVal,
        strafeVal
      ),
      rotationVal,
      true,
      true
    );

  }

  @Override
  public boolean isFinished() {
    return m_Intake.getObjectState();
  }
}
