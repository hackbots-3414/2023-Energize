package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class MoveWrist extends CommandBase {

  private Wrist wrist;
  private double speed;

  public MoveWrist(Wrist wrist, double speed) {
    this.wrist = wrist;
    this.speed = speed;
    addRequirements(wrist);

  }

  @Override
  public void initialize() {
    wrist.disable();
  }

  @Override
  public void execute() {
    wrist.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setSpeed(0.0);
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
