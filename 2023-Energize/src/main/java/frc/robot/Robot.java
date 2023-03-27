package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Robot extends TimedRobot {
  final static Logger logger = LoggerFactory.getLogger(Constants.class);
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

    DataLogManager.start();

        // UsbCamera camera = CameraServer.startAutomaticCapture();
        // camera.setFPS(60);
        // camera.setResolution(320, 240);
        setUpLimeLight();


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.getIntake().resetIntake();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.getSwerve().resetModulesToAbsolute();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}



public void setUpLimeLight() {
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // set pipeline for camera
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); // force off
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); // driver cam: turns off processing
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0); // Sets the display to side by side IF secondary camera is present.
  // NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setNumber(2);
}

}

