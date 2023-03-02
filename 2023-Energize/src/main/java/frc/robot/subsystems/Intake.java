package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Swerve;


public class Intake extends SubsystemBase {
  final static Logger logger = LoggerFactory.getLogger(Intake.class);
  // CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
  PowerDistribution powerDistribution = new PowerDistribution(Swerve.pdhID, ModuleType.kRev);
  TalonFX hand = new TalonFX(Constants.IntakeConstants.handMotorID);
  
  
  public Intake() {
    configMotor();
  }

  public void configMotor() {
    // hand.clearFaults();
    // hand.restoreFactoryDefaults();
    
    // hand.setInverted(Constants.IntakeConstants.handMotorInvert);
    // hand.setSmartCurrentLimit(Constants.IntakeConstants.handCurrentLimit);

    // hand.burnFlash();
    hand.configFactoryDefault();
    // hand.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    hand.setInverted(false);
    hand.setNeutralMode(NeutralMode.Coast);
    hand.setSelectedSensorPosition(0);
  }

  public void set(double speed) {
    hand.set(ControlMode.PercentOutput,speed);
  }

  public void stop() {
    hand.set(ControlMode.PercentOutput,0.0);
  }

  public double getCurrent() {
    return powerDistribution.getCurrent(4);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hand Motor Current", getCurrent());
  }

}
