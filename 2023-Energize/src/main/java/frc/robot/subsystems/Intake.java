package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  final static Logger logger = LoggerFactory.getLogger(Intake.class);
  // CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID,
  // MotorType.kBrushless);
  WPI_TalonFX hand = new WPI_TalonFX(IntakeConstants.handMotorID);
  private boolean hasObject = false;
  private boolean runningIntake = false;
  private MedianFilter currentFilter = new MedianFilter(5);// 9
  private double currentFilterValue = 0;

  public Intake() {
    configMotor();
  }

  private void configMotor() {
    hand.configFactoryDefault(IntakeConstants.canPause);
    hand.setSafetyEnabled(true);
    hand.setInverted(TalonFXInvertType.Clockwise);
    hand.setNeutralMode(NeutralMode.Brake);
    // hand.setNeutralMode(NeutralMode.Coast);
    // hand.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 14,
    // 0, 0), IntakeConstants.canPause);
    setCurrentLimitOne();
    hand.configOpenloopRamp(0.2, IntakeConstants.canPause);
  }

  public boolean getObjectState() {
    return hasObject;
  }

  public void setObjectStateTrue() {
    hasObject = true;
  }

public void setRunningIntake(boolean isRunning) {
  runningIntake = isRunning;
}

public boolean getRunningIntake() {
  return runningIntake;
}

  public void setObjectStateFalse() {
    hasObject = false;
  }

  public void setCurrentLimitOne() {
    // hand.clearFaults();
    // hand.setSmartCurrentLimit(Constants.IntakeConstants.handCurrentLimit);
    hand.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, Constants.IntakeConstants.handCurrentLimit, 0, 0),
        IntakeConstants.canPause);
    // hand.burnFlash();
  }

  public void setCurrentLimitTwo() {
    // hand.clearFaults();
    // hand.setSmartCurrentLimit(Constants.IntakeConstants.secondHandCurrentLimit);
    hand.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, Constants.IntakeConstants.secondHandCurrentLimit, 0, 0),
        IntakeConstants.canPause);
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
    // return hand.getSupplyCurrent();
    return currentFilterValue;
  }

  public void resetIntake() {
    hasObject = false;
    runningIntake = false;
  }

  @Override
  public void periodic() {
    hand.feed();
    // if (runningIntake) {
    //   intakeLogic();
    // }

    SmartDashboard.putNumber("Hand Motor Current", getCurrent());
    SmartDashboard.putBoolean("Has Object", hasObject);
    SmartDashboard.putBoolean("Running Intake", getRunningIntake());
    currentFilterValue = currentFilter.calculate(hand.getSupplyCurrent());
  }


  // public void intakeLogic() {
    
  //   if (!getObjectState()) {
  //     set(Constants.IntakeConstants.intakeSpeedPercent);

  //     if (getCurrent() > IntakeConstants.handCurrentThreshold) {
  //     setCurrentLimitTwo();
  //     setObjectStateTrue();
  //     set(IntakeConstants.objectHoldSpeedPercent);
  //     }
  //   }
  // }
}
