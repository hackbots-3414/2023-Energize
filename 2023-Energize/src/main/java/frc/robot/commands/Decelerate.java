// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IRSensor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Decelerate extends CommandBase {
  private Swerve swerve;
  private IRSensor irSensor;
  private double redSlowDownX = 601.21 / 39.3701;
  private double blueSlowDownX = 64.25 / 39.3701;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;
  private double multiplier;
  /** Creates a new Decelerate. */
  public Decelerate(Swerve swerve, IRSensor irSensor, DoubleSupplier translationSup, DoubleSupplier strafeSup,
  BooleanSupplier robotCentricSup) {

    addRequirements(swerve);
    this.swerve = swerve;
    this.irSensor = irSensor;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    double x = currentPose.getX();
    if (irSensor.getIRState()) {
      multiplier = 0;
    } else if (x > redSlowDownX || x < blueSlowDownX) {
      multiplier = 0.5;
    } else {
      multiplier = 1;
    }

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = 0;

    swerve.drive(
      new Translation2d(translationVal, strafeVal)
              .times(Constants.Swerve.maxSpeed * multiplier),
      rotationVal,
      !robotCentricSup.getAsBoolean(),
      true
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return irSensor.getIRState();
  }
}
