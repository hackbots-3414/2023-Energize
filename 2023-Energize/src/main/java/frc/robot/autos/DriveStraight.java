package frc.robot.autos;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveStraight extends CommandBase {
  private double speed;
  private double distance;
  private double direction;
  private double x;
  private double y;
  private Pose2d endPose;
  private Pose2d startPose;
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
    swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); // broken reset odometry, would reset to itself
          System.out.println(swerve.getPose().toString());
    this.x = Math.cos(direction); // speed multiplier, so removed distance multiplication
    this.y = Math.sin(direction);
    this.endPose = swerve.getPose().transformBy(new Transform2d(new Translation2d(x * distance, y * distance), new Rotation2d(0)));
    this.startPose = swerve.getPose();
    this.speed = Math.copySign(this.speed, this.distance);
  }

  @Override
  public void execute() {
    swerve.drive(new Translation2d(speed * x, speed * y), 0, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.zeroGyro();
  }

  @Override
  public boolean isFinished() {
    // SmartDashboard.putNumber("Change: ", startPose.getTranslation().getDistance(swerve.getPose().getTranslation()));
    return startPose.getTranslation().getDistance(swerve.translation2d) < distance ? false : true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
