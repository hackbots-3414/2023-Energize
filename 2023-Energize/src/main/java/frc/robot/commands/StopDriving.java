// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IRSensor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class StopDriving extends CommandBase {
  Swerve s_Swerve;
  //IRSensor m_IrSensor;
  Intake m_Intake;
  /** Creates a new StopDriving. */
  public StopDriving(Swerve s_Swerve, Intake m_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    this.s_Swerve = s_Swerve;
    //this.m_IrSensor = m_IrSensor;
    this.m_Intake = m_Intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.drive(new Translation2d(), 0, true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_IrSensor.isPickUpComplete();
    return m_Intake.getObjectState();
  }
}
