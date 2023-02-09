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

public class Wrist extends SubsystemBase {
  final static Logger logger = LoggerFactory.getLogger(Wrist.class);
  /** Creates a new Wrist. */
  WPI_TalonFX wrist = new WPI_TalonFX(IntakeConstants.wristMotorID);
  CANCoder wristCanCoder = new CANCoder(IntakeConstants.wristCanCoderID, Swerve.canbusString);

  public Wrist() {
    configMotor();
  }

  public void setSpeed(double speed) {
    wrist.set(speed);
  }

  private void configMotor(){        
    wrist.configFactoryDefault(IntakeConstants.canPause);
    wrist.configRemoteFeedbackFilter(wristCanCoder, 0, IntakeConstants.canPause);
    wrist.setSafetyEnabled(true);
  }

  public double getPosition(){
    // FIX ME USE CAN CODER
   return wrist.getSelectedSensorPosition();
  }

  public void stopWrist(){
    wrist.set(0.0);
  }

  @Override
  public void periodic() {
    wrist.feed();
    // This method will be called once per scheduler run
  }
}