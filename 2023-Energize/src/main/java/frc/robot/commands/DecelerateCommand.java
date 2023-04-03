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
  private double speedLimit;
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
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    double x = currentPose.getX();

    if (irSensor.getIRState()) {
      speedLimit = 0;
      // SmartDashboard.putBoolean("Infrared sensor", irSensor.getIRState());
      // System.out.println("Infrared sensor has been triggered!");
    } else if (x > Constants.IntakeAutomatic.redSideX || x < Constants.IntakeAutomatic.blueSideX) {
      speedLimit = Constants.IntakeAutomatic.slowLimit;
    } else {
      speedLimit = 1;
    }

    // Display values in SmartDashboard:
    SmartDashboard.putNumber("Distance to AprilTag (X)", x);
    SmartDashboard.putNumber("Limit", speedLimit);
    // SmartDashboard.putBoolean("Infrared sensor", irSensor.getIRState());


    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    SmartDashboard.putNumber("translationVal", translationVal);
    if (translationVal > speedLimit) {
      translationVal = speedLimit;
    }

    swerve.drive(
      new Translation2d(translationVal, strafeVal)
              .times(Constants.Swerve.maxSpeed),
      rotationVal,
      !robotCentricSup.getAsBoolean(),
      true
    );

  }

  @Override
  public boolean isFinished() {
    return irSensor.getIRState();
  }
}
