// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Shoulder;

public class IntakeAutomaticCommand extends IntakeCommand {

  final static Logger logger = LoggerFactory.getLogger(IntakeAutomaticCommand.class);

  Swerve swerve;
  Shoulder shoulder;
  Wrist wrist;
  boolean autoDrive = true;
  /*
   * false means drivver remains control over the robot
   * true means the driver aligns the robot then the code drives forward automatically.
   */

  /** Creates a new IntakeAutomaticCommand. */
  public IntakeAutomaticCommand(Swerve swerve, Intake intake, Shoulder shoulder, Wrist wrist) {
    super(intake);
    this.swerve = swerve;
    this.shoulder = shoulder;
    this.wrist = wrist;
    addRequirements(swerve);
    addRequirements(shoulder);
    addRequirements(wrist);
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
    // Step 1: Raise the arm:
    shoulder.setGoal(Constants.IntakeAngles.shelfShoulderAngle);

    // Step 2: Align the wrist:
    wrist.setGoal(Constants.IntakeAngles.shelfWristAngle);

    // Step 3: Start the wrist and shoulder:
    shoulder.enable();
    wrist.enable();

    // Step 4: Activate the intake:

    super.execute();
    
    // Auto-stop feature:

    // The IR sensor returns a boolean:
    /*
     * true means that we can keep going
     * false means that we can stop now because the sensor is triggered. It's kind of weird.
     */
    
    boolean shouldStopDriving = !super.intake.getIRState();

    if (shouldStopDriving) {
      swerve.driveForward(0, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (autoDrive) {
      swerve.driveForward(0,0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
