package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;

public class MoveShoulder extends CommandBase {

  WPI_TalonFX shoulder = new WPI_TalonFX(Constants.IntakeConstants.shoulderMotorID);

  private double rotationTarget;

  public MoveShoulder(double rotationTarget, Intake intake) {
    this.rotationTarget = rotationTarget;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}
    

  @Override
  public void end(boolean interrupted) {
    shoulder.set(0.0);
  }

  @Override
  public boolean isFinished() {
    double currentShoulderPosition = shoulder.getSelectedSensorPosition();
    if (Math.abs(currentShoulderPosition - rotationTarget) < 50) {
      return true;
    }
    return false;
  }

}
