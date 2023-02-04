package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class Intake extends SubsystemBase {
  final static Logger logger = LoggerFactory.getLogger(Intake.class);
  CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);

  DigitalInput irInput = new DigitalInput(9);

  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher = new ColorMatch();

  // private final Color cubeTarget = new Color(.168, .023, .178);
  // private final Color coneTarget = new Color( .235, .221, .011); 
  
  public Intake() {

  }

  public void spinHand(double speed) {
    hand.set(speed);
  }



  public void stopHand() {
    hand.set(0.0);
  }

  public boolean getIRInput() {
    return !irInput.get();
  }

  @Override
  public void periodic() {
    // Color detectedColor = m_colorSensor.getColor();

    // String colorString;
    // ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // if (match.color == cubeTarget) {
    // colorString = "Cube";
    // } else if (match.color == coneTarget) {
    // colorString = "Cone";
    // } else {
    // colorString = "Unknown";
    // }

    // SmartDashboard.putString("Detected Color", colorString);
  }

  // public void robotInit(){
  // m_colorMatcher.addColorMatch(cubeTarget);
  // m_colorMatcher.addColorMatch(coneTarget);
  // }

}
