// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class Wrist extends SubsystemBase {
  final static Logger logger = LoggerFactory.getLogger(Wrist.class);

  /** Creates a new Wrist. */
  private CANCoder wristCanCoder;
  private WPI_TalonFX wrist;


  public Wrist() {
    wristCanCoder = new CANCoder(IntakeConstants.wristCanCoderID);
    configWristEncoder();
    wrist = new WPI_TalonFX(IntakeConstants.wristMotorID);
    configMotor();
  }

  public void motionMagic(double angle) {
    wrist.set(ControlMode.MotionMagic, angle, DemandType.ArbitraryFeedForward, Constants.IntakeConstants.wristMaxGravFF * Math.cos(Math.toRadians(getCanCoder())));
  }

  public void setSpeed(double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }

  private void configMotor() {
    wrist.configFactoryDefault(IntakeConstants.canPause);
    wrist.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder(), Constants.IntakeConstants.wristGearRatio), 0, 100);
    wrist.configRemoteFeedbackFilter(wristCanCoder, 0, IntakeConstants.canPause);
    wrist.setSafetyEnabled(true);
    wrist.configForwardSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.wristUpperLimit, Constants.IntakeConstants.wristGearRatio), 100);
    wrist.configReverseSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.wristLowerLimit,Constants.IntakeConstants.wristGearRatio), 100);
    wrist.configForwardSoftLimitEnable(true, 100);
    wrist.configReverseSoftLimitEnable(true, 100);
    wrist.setInverted(TalonFXInvertType.CounterClockwise);
    wrist.setNeutralMode(NeutralMode.Brake);
  }

  private void configWristEncoder() {
    wristCanCoder.configFactoryDefault(Constants.IntakeConstants.canPause);
    wristCanCoder.configAllSettings(Robot.ctreConfigs.wristCanCoderConfig, IntakeConstants.canPause);
    wristCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, IntakeConstants.canPause);
    wristCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,IntakeConstants.canPause);
    wristCanCoder.configMagnetOffset(Constants.IntakeConstants.wristCanCoderOffset, IntakeConstants.canPause);
    wristCanCoder.configSensorDirection(Constants.IntakeConstants.wristCanCoderInvert, IntakeConstants.canPause);
  }

  public double getPosition() {
    // FIX ME USE CAN CODER
    return wrist.getSelectedSensorPosition();
  }

  public void stopWrist() {
    wrist.set(0.0);
  }

  public double getCanCoder() {
    return wristCanCoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    wrist.feed();
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist pos", getPosition());
    SmartDashboard.putNumber("Wrist CANCoder", getCanCoder());
  }
}
