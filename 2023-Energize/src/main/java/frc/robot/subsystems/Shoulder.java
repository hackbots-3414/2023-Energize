// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Swerve;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class Shoulder extends SubsystemBase {
  /** Creates a new Shoulder. */
  final static Logger logger = LoggerFactory.getLogger(Shoulder.class);
  WPI_TalonFX shoulder = new WPI_TalonFX(IntakeConstants.shoulderMotorID);
  CANCoder ShoulderCanCoder = new CANCoder(IntakeConstants.ShoulderCanCoderID, Swerve.canbusString);


  public Shoulder() {
    configMotor();
  }

  public void spin(double speed) {
    shoulder.set(speed);
  }

  public void stop() {
    shoulder.set(0.0);
  }

  private void configMotor() {
    shoulder.configFactoryDefault(IntakeConstants.canPause);
    shoulder.configRemoteFeedbackFilter(ShoulderCanCoder, 0, IntakeConstants.canPause);
    shoulder.setSafetyEnabled(true);
  }

  public double getPosition() {
    // FIXME USE CAN CODER
    return shoulder.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    shoulder.feed();
    // This method will be called once per scheduler run
  }
}
