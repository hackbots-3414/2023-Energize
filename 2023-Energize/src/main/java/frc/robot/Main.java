package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public final class Main {
  final static Logger logger = LoggerFactory.getLogger(Main.class);
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}