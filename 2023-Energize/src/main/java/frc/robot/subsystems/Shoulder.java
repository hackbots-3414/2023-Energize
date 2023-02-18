// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;


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

  public void motionMagic(double angle) {
    shoulder.set(ControlMode.MotionMagic, angle, DemandType.ArbitraryFeedForward, Constants.IntakeConstants.shoulderMaxGravFF * Math.cos(Math.toRadians(getCanCoder())));
  }

  public void set(double speed) {
    shoulder.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    shoulder.set(0.0);
  }
 
  private void configShoulderEncoder() {
    shoulderCanCoder.configFactoryDefault(Constants.IntakeConstants.canPause);
    shoulderCanCoder.configAllSettings(Robot.ctreConfigs.shoulderCanCoderConfig, IntakeConstants.canPause);
    shoulderCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, IntakeConstants.canPause);
    shoulderCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, IntakeConstants.canPause);
    shoulderCanCoder.configMagnetOffset(Constants.IntakeConstants.shoulderCanCoderOffset, IntakeConstants.canPause);
    shoulderCanCoder.configSensorDirection(Constants.IntakeConstants.shoulderCanCoderInvert, IntakeConstants.canPause);
}

  private void configMotor() {
    shoulder.configFactoryDefault(IntakeConstants.canPause);
    shoulder.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder(), Constants.IntakeConstants.shoulderGearRatio), 0, 100);
    shoulder.configRemoteFeedbackFilter(shoulderCanCoder, 0, IntakeConstants.canPause);
    shoulder.setSafetyEnabled(true);
    shoulder.configForwardSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.shoulderUpperLimit, Constants.IntakeConstants.shoulderGearRatio), 100);
    shoulder.configReverseSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.shoulderLowerLimit, Constants.IntakeConstants.shoulderGearRatio), 100);
    shoulder.configForwardSoftLimitEnable(true, 100);
    shoulder.configReverseSoftLimitEnable(true, 100);
    shoulder.setInverted(TalonFXInvertType.CounterClockwise);
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
  }
}
