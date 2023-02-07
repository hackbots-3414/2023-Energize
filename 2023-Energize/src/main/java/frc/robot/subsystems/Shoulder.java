// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Swerve;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class Shoulder extends SubsystemBase {
  /** Creates a new Shoulder. */

  
  final static Logger logger = LoggerFactory.getLogger(Shoulder.class);
  WPI_TalonFX shoulder = new WPI_TalonFX(IntakeConstants.shoulderMotorID);
  CANCoder ShoulderCanCoder = new CANCoder(IntakeConstants.ShoulderCanCoderID, Swerve.canbusString);


  public void moveShoulder(double angle) {
    
    shoulder.set(ControlMode.MotionMagic, angle);

  }

  public Shoulder() {
    configMotor();
  }

  public void spin(double speed) {
    shoulder.set(speed);
  }

  public void moveShoulderUp(double speed, int upperLimit) {

    double position = shoulder.getSelectedSensorPosition();
    while (position < upperLimit) {
      spin(speed);
    }

  }

  public void moveShoulderDown(double speed, int lowerLimit){
    double position = shoulder.getSelectedSensorPosition();

    while (position > lowerLimit) {
      spin(speed);
    }
  }

  public void stop() {
    shoulder.set(0.0);
  }

  private void configMotor() {
    shoulder.configFactoryDefault(IntakeConstants.canPause);
    shoulder.configRemoteFeedbackFilter(ShoulderCanCoder, 0, IntakeConstants.canPause);
    shoulder.setSafetyEnabled(true);
    shoulder.configForwardSoftLimitThreshold(Constants.IntakeConstants.shoulderLowerLimit, 0);
    shoulder.configReverseSoftLimitThreshold(Constants.IntakeConstants.shoulderUpperLimit, 0);
    shoulder.configForwardSoftLimitEnable(true, 0);
    shoulder.configReverseSoftLimitEnable(true, 0);
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
