package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class MoveWrist extends CommandBase {

  WPI_TalonFX wrist = new WPI_TalonFX(Constants.IntakeConstants.wristMotorID);

  private Intake intake;

  private double rotationTarget;

  public MoveWrist(double rotationTarget, Intake intake) {
    this.intake = intake;
    this.rotationTarget = rotationTarget;

    addRequirements(intake);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double currentWristPosition = wrist.getSelectedSensorPosition();
    if (currentWristPosition < rotationTarget) {
      wrist.set(0.20);
    }
    else if (currentWristPosition > rotationTarget) {
      wrist.set(-0.20);
    }
  }

  @Override
  public void end(boolean interrupted) {
    wrist.set(0.0);
    
  }

  @Override
  public boolean isFinished() {
    double currentWristPosition = wrist.getSelectedSensorPosition();
    if (Math.abs(currentWristPosition - rotationTarget) < 50) {
      return true;
    }
    return false;
  }

}
