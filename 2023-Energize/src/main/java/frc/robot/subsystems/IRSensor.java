// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IRSensor extends SubsystemBase {

  final static Logger logger = LoggerFactory.getLogger(IRSensor.class);

  private DigitalInput irSensor = new DigitalInput(0);
  private boolean isPickUpComplete = false;
  private double startTime;

  private boolean irSensorState;
  /** Creates a new IRSensor. */
  public IRSensor() {}

  public boolean isPickUpComplete() {
    return isPickUpComplete;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    irSensorState = irSensor.get();
    SmartDashboard.putBoolean("Infrared sensor", irSensorState);
    if (getIRState() && !isPickUpComplete) {
      isPickUpComplete = true;
      //Start timer
      startTime = System.currentTimeMillis();
      logger.warn("IR Sensor Triggered");

    }
    if ((System.currentTimeMillis() - startTime)/1000 >= 1) {
      isPickUpComplete = false;
    }
  }
  
  public boolean getIRState() {
    return !irSensorState;
  }
}
