// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class Shoulder extends ProfiledPIDSubsystem {
  /** Creates a new Shoulder. */

  final static Logger logger = LoggerFactory.getLogger(Shoulder.class);

  WPI_TalonFX shoulder = new WPI_TalonFX(IntakeConstants.shoulderMotorID);
  CANCoder shoulderCanCoder = new CANCoder(IntakeConstants.ShoulderCanCoderID);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      IntakeConstants.shoulderkS, IntakeConstants.shoulderkG,
      IntakeConstants.shoulderkV, IntakeConstants.shoulderkA);

  public Shoulder() {
    super(
      new ProfiledPIDController(
        IntakeConstants.shoulderkP, 
        0, 
        0, 
        new TrapezoidProfile.Constraints(
          IntakeConstants.shouldermaxVelo, 
          IntakeConstants.shouldermaxAccel)), 
        0);

    configShoulderEncoder();
    Timer.delay(0.1);
    configMotor();

    m_controller.reset(getMeasurement(), getCanCoderVelo());
    m_controller.enableContinuousInput(IntakeConstants.shoulderLowerLimit, IntakeConstants.shoulderUpperLimit);
    SmartDashboard.putData(m_controller);
    setGoal(getMeasurement());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //calculate feedforward from setpoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    //add the feedforward to the PID output to get the motor output
    shoulder.setVoltage(output + feedforward);
    // System.out.println("FeedForward: " + (output + feedforward));
  }

  @Override
  public double getMeasurement() {
    return Math.toRadians(getCanCoder());
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
    shoulderCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,
        IntakeConstants.canPause);
    shoulderCanCoder.configMagnetOffset(Constants.IntakeConstants.shoulderCanCoderOffset, IntakeConstants.canPause);
    shoulderCanCoder.configSensorDirection(Constants.IntakeConstants.shoulderCanCoderInvert, IntakeConstants.canPause);
  }

  private void configMotor() {
    shoulder.configFactoryDefault(IntakeConstants.canPause);
    shoulder.setSelectedSensorPosition(
        Conversions.degreesToFalcon(getCanCoder(), Constants.IntakeConstants.shoulderGearRatio), 0, 100);
    shoulder.setSafetyEnabled(true);
    shoulder.configForwardSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.shoulderUpperLimit,
        Constants.IntakeConstants.shoulderGearRatio), 100);
    shoulder.configReverseSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.shoulderLowerLimit,
        Constants.IntakeConstants.shoulderGearRatio), 100);
    shoulder.configForwardSoftLimitEnable(true, 100);
    shoulder.configReverseSoftLimitEnable(true, 100);
    shoulder.setInverted(TalonFXInvertType.CounterClockwise);
  shoulder.setNeutralMode(NeutralMode.Brake);
    //  shoulder.setNeutralMode(NeutralMode.Coast);
    shoulder.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 14, 0, 0), IntakeConstants.canPause);
  }

  public double getPosition() {
    // FIXME USE CAN CODER
    return shoulder.getSelectedSensorPosition();
  }

  public double getCanCoder() {
    return shoulderCanCoder.getAbsolutePosition();
  }

  public double getCanCoderVelo() {
    return Math.toRadians(shoulderCanCoder.getVelocity());
  }

  @Override
  public void periodic() {
    super.periodic();
    shoulder.feed();
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shoulder pos", getPosition());

    SmartDashboard.putNumber("Shoulder CANCoder", getCanCoder());
    // SmartDashboard.putNumber("Shoulder Velo", Math.toDegrees(getCanCoderVelo()));
    SmartDashboard.putNumber("Shoulder Current", shoulder.getSupplyCurrent());
  }
}
