package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
 //CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
 WPI_TalonFX hand = new WPI_TalonFX(IntakeConstants.handMotorID);
  private boolean hasObject = false;
  
  public Intake() {
    configMotor();
  }

  private void configMotor() {
    hand.configFactoryDefault(IntakeConstants.canPause);
    hand.setSafetyEnabled(true);
    hand.setInverted(TalonFXInvertType.CounterClockwise);
  hand.setNeutralMode(NeutralMode.Brake);
    //  hand.setNeutralMode(NeutralMode.Coast);
    hand.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 14, 0, 0), IntakeConstants.canPause);
  }

  public boolean getObjectState() {
    return hasObject;
  }

  public void setObjectStateTrue() {
    hasObject = true;
  }

  public void setObjectStateFalse() {
    hasObject = false;
  }

  public void setCurrentLimitOne() {
    //hand.clearFaults();
      //hand.setSmartCurrentLimit(Constants.IntakeConstants.handCurrentLimit);
      hand.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.IntakeConstants.handCurrentLimit, 0, 0), IntakeConstants.canPause);
  //hand.burnFlash();
  }

  public void setCurrentLimitTwo() {
    //hand.clearFaults();
    //hand.setSmartCurrentLimit(Constants.IntakeConstants.secondHandCurrentLimit);
    hand.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.IntakeConstants.handCurrentLimit, 0, 0), IntakeConstants.canPause);
   // hand.burnFlash();
  }

  public void set(double speed) {
    hand.set(speed);
  }

  public void stop() {
    hand.set(0.0);
  }

  public double getCurrent() {
    // return powerDistribution.getCurrent(13);
    return hand.getSupplyCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hand Motor Current", getCurrent());
    SmartDashboard.putBoolean("Has Object", hasObject);
    SmartDashboard.putNumber("Hand Motor Temp Degrees", (hand.getTemperature() * (5.0/9.0)) + 32);
    SmartDashboard.putNumber("celcius", hand.getTemperature());

  }

}
