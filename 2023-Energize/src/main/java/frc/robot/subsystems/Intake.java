package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  CANSparkMax hand = new CANSparkMax(Constants.IntakeConstants.handMotorID, MotorType.kBrushless);
  WPI_TalonFX wrist = new WPI_TalonFX(Constants.IntakeConstants.wristMotorID, Constants.Swerve.canbusString);
  WPI_TalonFX shoulder = new WPI_TalonFX(Constants.IntakeConstants.shoulderMotorID, Constants.Swerve.canbusString);
  // CANCoder wristCanCoder = new CANCoder(Constants.IntakeConstants.wristCanCoderID, Constants.Swerve.canbusString);
  // CANCoder ShoulderCanCoder = new CANCoder(Constants.IntakeConstants.ShoulderCanCoderID, Constants.Swerve.canbusString);

  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher = new ColorMatch();

  // private final Color cubeTarget = new Color(.168, .023, .178);
  // private final Color coneTarget = new Color( .235, .221, .011); 
  
  public Intake() {}

  public void spinAll(double speed) {
    hand.set(speed);
    wrist.set(ControlMode.Velocity, speed);
    shoulder.set(ControlMode.Velocity, speed);
  }

  public void stopAll() {
    hand.set(0);
    wrist.set(ControlMode.Velocity, 0);
    shoulder.set(ControlMode.Velocity, 0);
  }

  @Override
  public void periodic() {
    // Color detectedColor = m_colorSensor.getColor();

    // String colorString;
    // ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // if (match.color == cubeTarget) {
    //   colorString = "Cube";
    // } else if (match.color == coneTarget) {
    //   colorString = "Cone";
    // } else {
    //   colorString = "Unknown";
    // }

    // SmartDashboard.putString("Detected Color", colorString);
  }

  // public void robotInit(){
  //   m_colorMatcher.addColorMatch(cubeTarget);
  //   m_colorMatcher.addColorMatch(coneTarget);
  // }
}
