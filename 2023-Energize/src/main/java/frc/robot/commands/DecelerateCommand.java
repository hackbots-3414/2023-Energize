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
import frc.robot.Constants.IntakeAngles;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IRSensor;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;

public class DecelerateCommand extends CommandBase {
  private Swerve swerve;
  private IRSensor irSensor;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private Shoulder shoulder;
  /** Creates a new Decelerate. */
  public DecelerateCommand(Swerve swerve, IRSensor irSensor, DoubleSupplier translationSup, DoubleSupplier strafeSup,
  DoubleSupplier rotationSup,
  BooleanSupplier robotCentricSup, Shoulder shoulder) {

    addRequirements(swerve);
    this.swerve = swerve;
    this.irSensor = irSensor;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.shoulder = shoulder;
  }

  @Override
  public void execute() {
    // Pose2d currentPose = swerve.getPose();
    // double x = currentPose.getX();

    

    // Display values in SmartDashboard:
    // SmartDashboard.putNumber("Distance to AprilTag (X)", x);
    // SmartDashboard.putNumber("Limit", speedLimit);
    // SmartDashboard.putBoolean("Infrared sensor in Dec", irSensor.getIRState());


    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    if (shoulder.getCanCoder() > Constants.IntakeAngles.midShoulderAngle - 5) {
      rotationVal *= Constants.IntakeConstants.slowTurn;
    }

    // SmartDashboard.putNumber("translationVal", translationVal);
    
    // if (shoulder.getCanCoder() > Constants.IntakeAngles.shelfShoulderAngle - 15) {
    //   translationVal *= speedLimit;
    // }

    swerve.drive(
      new Translation2d(translationVal, strafeVal)
              .times(Constants.Swerve.maxSpeed),
      rotationVal,
      !robotCentricSup.getAsBoolean(),
      true
    );
    
  }
  // TODO: Move this code into end:
  // SmartDashboard.putNumber("translationVal", translationVal);
    
    // if (shoulder.getCanCoder() > Constants.IntakeAngles.shelfShoulderAngle - 15) {
    //   translationVal *= speedLimit;
    // }  

  // @Override
  // public void end(boolean isInterrupted) {
  //   swerve.drive(new Translation2d(0,0), 0, true, true);
  // }

  @Override
  public boolean isFinished() {
    return (shoulder.getCanCoder() > Constants.IntakeAngles.shelfShoulderAngle - 15) && irSensor.getIRState();
  }
}
