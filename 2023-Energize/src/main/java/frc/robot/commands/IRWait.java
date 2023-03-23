// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IRSensor;

public class IRWait extends CommandBase {

  final static Logger logger = LoggerFactory.getLogger(IRWait.class);

  IRSensor intake;
  boolean autoDrive = true;
  /*
   * false means drivver remains control over the robot
   * true means the driver aligns the robot then the code drives forward automatically.
   */

  /** Creates a new IntakeAutomaticCommand. */
  public IRWait(IRSensor intake) {
    addRequirements(intake);
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
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
    return intake.getIRState();
  }
}
