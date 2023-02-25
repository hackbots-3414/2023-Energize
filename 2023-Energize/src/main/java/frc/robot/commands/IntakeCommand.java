package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;



public class IntakeCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(IntakeCommand.class);

  final Intake intake;
  private static boolean finished = false;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    DataLogManager.start();
    intake.set(Constants.IntakeConstants.intakeSpeedPercent);
  }

  @Override
  public void execute() {
    // if (intake.getCurrent() > IntakeConstants.handCurrentThreshold) {
    //   finished = true;
    //   System.err.println(123234);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
