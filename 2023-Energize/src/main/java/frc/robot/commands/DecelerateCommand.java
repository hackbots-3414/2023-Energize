// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IRSensor;
import frc.robot.subsystems.Swerve;

public class DecelerateCommand extends CommandBase {
  private Swerve swerve;
  private IRSensor irSensor;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private double multiplier;
  /** Creates a new Decelerate. */
  public DecelerateCommand(Swerve swerve, IRSensor irSensor, DoubleSupplier translationSup, DoubleSupplier strafeSup,
  DoubleSupplier rotationSup,
  BooleanSupplier robotCentricSup) {

    addRequirements(swerve);
    this.swerve = swerve;
    this.irSensor = irSensor;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
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
      System.out.println("Infared sensor has been triggered!");
      //TODO: Ensure that these variables are stored in Constants where we want them.
    } else if (x > Constants.IntakeAutomatic.redSlowDownX || x < Constants.IntakeAutomatic.blueSlowDownX) {
      multiplier = 0.2; // TODO: Find a proper number for this
      System.out.println("AprilTag Boundary crossed");
    } else {
      multiplier = 1;
    }

    // Display values in SmartDashboard:
    SmartDashboard.putNumber("Distance to AprilTag (X)", x);
    SmartDashboard.putNumber("Multiplier", multiplier);

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

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
