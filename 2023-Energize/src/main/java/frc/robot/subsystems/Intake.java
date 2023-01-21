package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
  WPI_TalonFX wrist = new WPI_TalonFX(Constants.IntakeConstants.wristMotorID, Constants.Swerve.canbusString);
  WPI_TalonFX shoulder = new WPI_TalonFX(Constants.IntakeConstants.shoulderMotorID, Constants.Swerve.canbusString);
  CANCoder wristCanCoder = new CANCoder(Constants.IntakeConstants.wristCanCoderID, Constants.Swerve.canbusString);
  CANCoder ShoulderCanCoder = new CANCoder(Constants.IntakeConstants.ShoulderCanCoderID, Constants.Swerve.canbusString);
  
  public Intake() {}

  @Override
  public void periodic() {}
}
