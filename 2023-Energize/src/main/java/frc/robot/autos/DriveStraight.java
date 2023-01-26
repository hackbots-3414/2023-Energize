package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveStraight extends CommandBase {
  private double speed;
  private double distance;
  private double direction;
  private double x;
  private double y;
  private double originalEncoders;
  private final double requestedDistance;
  private final Swerve swerve;
  

  public DriveStraight(Swerve subsystem, double distance, double direction) {
    this.distance = distance;
    this.speed = 0.6;
    this.requestedDistance = distance;
    this.direction = direction;
    swerve = subsystem;
  }

  public DriveStraight(Swerve subsystem, double distance, double speed, double direction) {
    this.distance = distance;
    this.speed = speed;
    this.requestedDistance = distance;
    this.direction = direction;

    swerve = subsystem;
  }

  @Override
  public void initialize() {
    this.distance = requestedDistance;
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.zeroGyro();

    this.x = Math.cos(direction) * distance;
    this.y = Math.sin(direction) * distance;
    this.speed = Math.copySign(this.speed, this.distance);
    this.distance = Math.abs(this.distance / Constants.Swerve.inchesPerTick); // converts from inches to motor ticks (wheel diameter 6.432 inches) (old 0.00083101561761)
    originalEncoders = swerve.getAverageSensorPositions();
  }

  @Override
  public void execute() {
    swerve.drive(new Translation2d(speed * x, speed * y), 0, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(speed * -1 * x, speed * -1 * y), 0, true, true);
    Timer.delay(0.01);
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.zeroGyro();
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(swerve.getAverageSensorPositions() - originalEncoders) * Constants.Swerve.distanceToTicks <= this.distance) {
      // System.out.println(Math.abs(swerve.getAverageSensorPositions() - originalEncoders) * Constants.Swerve.distanceToTicks);
      return false;
    } else {
        return true;
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
