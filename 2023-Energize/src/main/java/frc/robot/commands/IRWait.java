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

  public IRWait(IRSensor intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public boolean isFinished() {
    return intake.getIRState();
  }
}
