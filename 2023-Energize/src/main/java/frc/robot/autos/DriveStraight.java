package frc.robot.autos;

import com.fasterxml.jackson.core.TreeNode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveStraight extends CommandBase {
  private double speed;
  private double distance;
  private double direction;
  private double x;
  private double y;
  private Pose2d startPose;
  private Translation2d translation2d;
  private final Swerve swerve;
  

  public DriveStraight(Swerve subsystem, double distance, double direction) {
    this.distance = distance;
    this.speed = 0.6;
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
    this.startPose = swerve.getPose();
    this.speed = Math.copySign(this.speed, this.distance);
    translation2d = new Translation2d(speed * x, speed * y);
  }

  @Override
  public void execute() {
    swerve.drive(translation2d, 0, true, true);
    Timer.delay(0.1);
  }

  @Override
  public void end(boolean interrupted) {
    //swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.zeroGyro();
  }

  @Override
  public boolean isFinished() {
    //System.out.println("Current Distance: " + startPose.getTranslation().getDistance(swerve.translation2d));
    return startPose.getTranslation().getDistance(swerve.translation2d) < distance ? false : true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}