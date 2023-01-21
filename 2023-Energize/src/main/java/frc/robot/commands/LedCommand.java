// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LedCommand extends CommandBase {
  /** Creates a new LedCommand. */
  private LedSubsystem m_LedSubsystem;
  final IntakeSubsystem m_Intake; 

  public LedCommand(LedSubsystem m_LedSubsystem, IntakeSubsystem m_Intake) {
    this.m_LedSubsystem = m_LedSubsystem;
    this.m_Intake = m_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LedSubsystem.setColor(0.53);
    if (m_Intake.getIRInput()) {
m_LedSubsystem.setColor(0.55);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
