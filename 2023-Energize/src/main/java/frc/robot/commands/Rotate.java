package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Rotate extends CommandBase {
  final Swerve swerve;
  public double startingGyro;
  public Rotate(Swerve s_swerve) {
    this.swerve = s_swerve;
    addRequirements(s_swerve);
  }

  @Override
  public void initialize() {
    swerve.zeroGyro();
    startingGyro = swerve.getYaw().getDegrees();
  }

  @Override
  public void execute() {
    swerve.drive(new Translation2d(), .5, true, true);
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(swerve.getYaw().getDegrees()-startingGyro) > 179){
      return true;
    } 
    return false;
  }
}
