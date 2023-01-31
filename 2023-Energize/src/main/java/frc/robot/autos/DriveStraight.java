package frc.robot.autos;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveStraight extends CommandBase {
  private double speed;
  private double distance;
  private double direction;
  private double x;
  private double y;
  private Pose2d translatedPose;
  private final Swerve swerve;
  

  public DriveStraight(Swerve subsystem, double distance, double direction) {
    System.out.println("CONSTRUCTOR");
    this.distance = distance;
    this.speed = 1;
    this.direction = direction;
    swerve = subsystem;
  }

  public DriveStraight(Swerve subsystem, double distance, double speed, double direction) {
    this.distance = distance;
    this.speed = speed;
    this.direction = direction;
    swerve = subsystem;
  }

  @Override
  public void initialize() {
    System.out.println("INITIALIZE");
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.zeroGyro();

    this.translatedPose = swerve.getPose().transformBy(new Transform2d(new Translation2d(Math.cos(direction) * distance, Math.sin(direction) * distance), new Rotation2d(0)));

    this.x = Math.cos(direction) * distance;
    this.y = Math.sin(direction) * distance;
    this.speed = Math.copySign(this.speed, this.distance);
    this.distance = Math.abs(this.distance / Constants.Swerve.distanceToTicks);
  }

  @Override
  public void execute() {
    System.out.println("EXECUTE");
    swerve.drive(new Translation2d(speed * x, speed * y), 0, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("END");
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.zeroGyro();
  }

  @Override
  public boolean isFinished() {
    System.out.println("ISFINISHED");
    System.out.println("New Pose: " + translatedPose.getTranslation().getNorm() + "; Current Pose: " + swerve.getPose().getTranslation().getNorm());
    System.out.println("New Pose: " + translatedPose.getTranslation().toString() + "; Current Pose: " + swerve.getPose().getTranslation().toString());
    return translatedPose.getTranslation().getNorm() > swerve.getPose().getTranslation().getNorm() ? false : true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
