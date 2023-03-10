package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Swerve;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
public class Intake extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  DigitalInput irSensor = new DigitalInput(0);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  final static Logger logger = LoggerFactory.getLogger(Intake.class);
  CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
  PowerDistribution powerDistribution = new PowerDistribution(Swerve.pdhID, ModuleType.kRev);
  private boolean hasObject = false;
  
  public Intake() {
    configMotor();
  }
  public boolean getirSensor() {
    return !irSensor.get();
  }
  public void robotinit() {
    m_colorMatcher.addColorMatch(kYellowTarget);
  }
  public void configMotor() {
    hand.clearFaults();
    hand.restoreFactoryDefaults();
    
    hand.setInverted(Constants.IntakeConstants.handMotorInvert);
    hand.setSmartCurrentLimit(Constants.IntakeConstants.handCurrentLimit);

    hand.burnFlash();
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
    hand.clearFaults();
    hand.setSmartCurrentLimit(Constants.IntakeConstants.handCurrentLimit);
    hand.burnFlash();
  }

  public void setCurrentLimitTwo() {
    hand.clearFaults();
    hand.setSmartCurrentLimit(Constants.IntakeConstants.secondHandCurrentLimit);
    hand.burnFlash();
  }

  public void set(double speed) {
    hand.set(speed);
  }

  public void stop() {
    hand.set(0.0);
  }

  public double getCurrent() {
    // return powerDistribution.getCurrent(13);
    return hand.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hand Motor Current", getCurrent());
    SmartDashboard.putBoolean("Has Object", hasObject);
    SmartDashboard.putNumber("Hand Motor Temp Degrees", (hand.getMotorTemperature() * (5.0/9.0)) + 32);
    SmartDashboard.putNumber("celcius", hand.getMotorTemperature());
    SmartDashboard.putBoolean("IR Sensor", getirSensor());
  }

}
