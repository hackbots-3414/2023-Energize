// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Swerve;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class Shoulder extends SubsystemBase {
  /** Creates a new Shoulder. */

  
  final static Logger logger = LoggerFactory.getLogger(Shoulder.class);
  WPI_TalonFX shoulder;
  CANCoder shoulderCanCoder;
 
  public Shoulder() {

    shoulderCanCoder = new CANCoder(IntakeConstants.ShoulderCanCoderID);
    configShoulderEncoder();

    shoulder = new WPI_TalonFX(IntakeConstants.shoulderMotorID);
    configMotor();
    
  }
 
  private void configShoulderEncoder() {
    shoulderCanCoder.configFactoryDefault(Constants.IntakeConstants.canPause);
    shoulderCanCoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
}

  public void moveShoulder(double angle) {
    
    shoulder.set(ControlMode.MotionMagic, angle);

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
    shoulder.configRemoteFeedbackFilter(shoulderCanCoder, 0, IntakeConstants.canPause);
    shoulder.setSafetyEnabled(true);
    shoulder.configForwardSoftLimitThreshold(Constants.IntakeConstants.shoulderLowerLimit, 0);
    shoulder.configReverseSoftLimitThreshold(Constants.IntakeConstants.shoulderUpperLimit, 0);
    shoulder.configForwardSoftLimitEnable(false, 0);
    shoulder.configReverseSoftLimitEnable(false, 0);
    shoulder.setNeutralMode(NeutralMode.Brake);
  }

  public double getPosition() {
    // FIXME USE CAN CODER
    return shoulder.getSelectedSensorPosition();
  }

  public double getCanCoder() {
    return shoulderCanCoder.getAbsolutePosition();
}

  @Override
  public void periodic() {
    shoulder.feed();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder pos", getPosition());
    SmartDashboard.putNumber("Shoulder CANCoder", getCanCoder());
  }
}
