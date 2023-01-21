package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
  WPI_TalonFX wrist = new WPI_TalonFX(Constants.IntakeConstants.wristMotorID, Constants.Swerve.canbusString);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color cubeTarget = new Color(.168, .023, .178);
  private final Color coneTarget = new Color( .235, .221, .011); 

  public void robotInit(){
    m_colorMatcher.addColorMatch(cubeTarget);
    m_colorMatcher.addColorMatch(coneTarget);
  }

  public Intake() {}

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == cubeTarget) {
      colorString = "Cube";
    } else if (match.color == coneTarget) {
      colorString = "Cone";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putString("Detected Color", colorString);
    
  }
}
