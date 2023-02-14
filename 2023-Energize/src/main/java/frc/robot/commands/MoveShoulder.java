package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;

public class MoveShoulder extends CommandBase {

  WPI_TalonFX shoulder = new WPI_TalonFX(Constants.IntakeConstants.shoulderMotorID);

  private double rotationTarget;

  public MoveShoulder(Shoulder shoulder) {
    addRequirements(shoulder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shoulder.set(5);
  }
    

  @Override
  public void end(boolean interrupted) {
    shoulder.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
