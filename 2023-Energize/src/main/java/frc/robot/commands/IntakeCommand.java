package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(IntakeCommand.class);

  final Intake intake;
  private int bounces;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setCurrentLimitOne();
    DataLogManager.start();
    intake.set(Constants.IntakeConstants.intakeSpeedPercent);
    bounces = 0;


  }

  @Override
  public void execute() {
    if (intake.getCurrent() > IntakeConstants.handCurrentThreshold) {
      bounces++;
    }

    if (!intake.getObjectState()) {
      intake.set(Constants.IntakeConstants.intakeSpeedPercent);
      
      if (bounces >= 9) {
        intake.setCurrentLimitTwo();
        intake.setObjectStateTrue();
        intake.set(IntakeConstants.objectHoldSpeedPercent);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!intake.getObjectState()) {
      intake.set(0.0);
    }
    intake.setCurrentLimitOne();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
