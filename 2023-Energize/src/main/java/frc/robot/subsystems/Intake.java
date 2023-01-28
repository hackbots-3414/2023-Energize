package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
  WPI_TalonFX wrist = new WPI_TalonFX(Constants.IntakeConstants.wristMotorID);
  WPI_TalonFX shoulder = new WPI_TalonFX(Constants.IntakeConstants.shoulderMotorID);
  // CANCoder wristCanCoder = new CANCoder(Constants.IntakeConstants.wristCanCoderID, Constants.Swerve.canbusString);
  // CANCoder ShoulderCanCoder = new CANCoder(Constants.IntakeConstants.ShoulderCanCoderID, Constants.Swerve.canbusString);

  DigitalInput irInput = new DigitalInput(9);

  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher = new ColorMatch();

  // private final Color cubeTarget = new Color(.168, .023, .178);
  // private final Color coneTarget = new Color( .235, .221, .011); 
  
  public Intake() {
    configWristMotor();
    configShoulderMotor();
  }

  public void spinAll(double speed) {
    hand.set(speed);
    wrist.set(speed);
    shoulder.set(speed);
  }

  public void stopAll() {
    hand.set(0);
    wrist.set(0);
    shoulder.set(0);
  }

  public void spinWrist(double speed) {
    wrist.set(speed);
  }

  public void spinHand(double speed) {
    hand.set(speed);
  }

  public void spinShoulder(double speed) {
    shoulder.set(speed);
  }

  public void stopWrist() {
    wrist.set(0.0);
  }

  public void stopHand() {
    hand.set(0.0);
  }

  public void stopShoulder() {
    shoulder.set(0.0);
  }

  private void configWristMotor(){        
    wrist.configFactoryDefault();
  }

  private void configShoulderMotor() {
    shoulder.configFactoryDefault();
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
  public double getShoulderPosition() {
    // FIXME USE CAN CODER

    return shoulder.getSelectedSensorPosition();

  }
public double getWristPosition(){
  // FIX ME USE CAN CODER

  return wrist.getSelectedSensorPosition();
}

}
