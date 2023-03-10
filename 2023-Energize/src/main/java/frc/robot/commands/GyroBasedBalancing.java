package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autos.ManualPathweaver;
import frc.robot.subsystems.Swerve;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DataLogManager;




// have to incorporate feature that makes it perpendicular 
// to the charging station incase the drivers moved sideways


public class GyroBasedBalancing extends CommandBase {
  final static Logger logger = LoggerFactory.getLogger(GyroBasedBalancing.class);
  /** Creates a new gyroBasedBalancing. */
  public Rotation2d yaw;
  public Rotation2d pitch;
  public Rotation2d roll;
  public double netBalance;
  public double goal = 1;
  public double current;
  public double previous;
  public Swerve swerve;
  public double prev;
  public double multiplier = 1;

  public GyroBasedBalancing(Swerve swerveInput) {
    this.swerve = swerveInput;
    yaw = swerve.getYaw(); 
    pitch = swerve.getPitch();
    roll = swerve.getRoll();
    netBalance = Math.sqrt(Math.pow(pitch.getDegrees(), 2) + Math.pow(roll.getDegrees(), 2));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.start();
    current = netBalance;
    prev = netBalance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("IN EXECUTE");
    yaw = swerve.getYaw(); 
    pitch = swerve.getPitch();
    roll = swerve.getRoll();
    SmartDashboard.putNumber("Pitch", pitch.getDegrees());
    SmartDashboard.putNumber("Roll", roll.getDegrees());
    current = Math.sqrt(Math.pow(pitch.getDegrees(), 2) + Math.pow(roll.getDegrees(), 2));
    SmartDashboard.putNumber("Combined", current);
    new InstantCommand(() -> new ManualPathweaver(swerve, 1, 0, 0));
    if (current > prev) {
      multiplier = -1;
    }
    prev = current;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return current < goal ? true : false;
  }
}
