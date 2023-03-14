// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class Wrist extends ProfiledPIDSubsystem {
  final static Logger logger = LoggerFactory.getLogger(Wrist.class);

  private Shoulder m_Shoulder;

  /** Creates a new Wrist. */
  private CANCoder wristCanCoder = new CANCoder(IntakeConstants.wristCanCoderID);;
  private WPI_TalonFX wrist = new WPI_TalonFX(IntakeConstants.wristMotorID);
  private double wristkS = IntakeConstants.wristkS;
  private double wristkG = IntakeConstants.wristkG;
  private double wristkV = IntakeConstants.wristkV;
  private double wristkA = IntakeConstants.wristkA;

  private ArmFeedforward m_feedforward = new ArmFeedforward(
      IntakeConstants.wristkS, IntakeConstants.wristkG,
      IntakeConstants.wristkV, IntakeConstants.wristkA);

  public Wrist(Shoulder m_Shoulder) {
    super(
        new ProfiledPIDController(
            IntakeConstants.wristkP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                IntakeConstants.wristmaxVelo,
                IntakeConstants.wristmaxAccel)),
        0.0);

        this.m_Shoulder = m_Shoulder;

        SmartDashboard.putData("WristPID", m_controller);
        SmartDashboard.putNumber("wristkA: ", wristkA);
        SmartDashboard.putNumber("wristkS: ", wristkS);
        SmartDashboard.putNumber("wristkG: ", wristkG);
        SmartDashboard.putNumber("wristkV: ", wristkV);

    configWristEncoder();
    Timer.delay(1);
    configMotor();

    m_controller.reset(getMeasurement(), getCanCoderVelo());
  }

  public void setSpeed(double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }

  private void configMotor() {
    wrist.configFactoryDefault(IntakeConstants.canPause);
    wrist.setSelectedSensorPosition(
        Conversions.degreesToFalcon(getCanCoder(), Constants.IntakeConstants.wristGearRatio), 0, 100);
    wrist.setSafetyEnabled(true);
    wrist.configForwardSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.wristUpperLimit,
        Constants.IntakeConstants.wristGearRatio), 100);
    wrist.configReverseSoftLimitThreshold(Conversions.degreesToFalcon(Constants.IntakeConstants.wristLowerLimit,
        Constants.IntakeConstants.wristGearRatio), 100);
    wrist.configForwardSoftLimitEnable(false, 100);
    wrist.configReverseSoftLimitEnable(false, 100);
    // wrist.setInverted(TalonFXInvertType.CounterClockwise);
    wrist.setNeutralMode(NeutralMode.Brake);
    //  wrist.setNeutralMode(NeutralMode.Coast);
    wrist.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 14, 0, 0), IntakeConstants.canPause);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    wristkA = SmartDashboard.getNumber("wristkA: ", wristkA);
    wristkS = SmartDashboard.getNumber("wristkS: ", wristkS);
    wristkG = SmartDashboard.getNumber("wristkG: ", wristkG);
    wristkV = SmartDashboard.getNumber("wristkV: ", wristkV);
    m_feedforward = new ArmFeedforward(wristkS, wristkG, wristkV, wristkA);

    // calculate feedforward from setpoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // add the feedforward to the PID output to get the motor output
    wrist.setVoltage(output + feedforward);
    // System.out.println("FeedForward: " + (output + feedforward));
  }

  @Override
  public double getMeasurement() {
    return Math.toRadians(getWristAngle());
  }

  public double getCanCoderVelo() {
    return Math.toRadians(wristCanCoder.getVelocity());
  }

  private void configWristEncoder() {
    wristCanCoder.configFactoryDefault(Constants.IntakeConstants.canPause);
    wristCanCoder.configAllSettings(Robot.ctreConfigs.wristCanCoderConfig, IntakeConstants.canPause);
    wristCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, IntakeConstants.canPause);
    wristCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,
        IntakeConstants.canPause);
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

  public double getWristAngle() {
    return getCanCoder() + getShoulderCanCoder();
  }

  private double getShoulderCanCoder() {
    return m_Shoulder.getCanCoder();
  }

  @Override
  public void periodic() {
    super.periodic();
    wrist.feed();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist pos", getPosition());
    SmartDashboard.putNumber("Wrist CANCoder", getCanCoder());
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
  }
}
