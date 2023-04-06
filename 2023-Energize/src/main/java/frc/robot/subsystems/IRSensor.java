// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IRSensor extends SubsystemBase {

  private DigitalInput irSensor = new DigitalInput(0);
  private boolean isPickUpComplete = false;
  private double startTime;
  /** Creates a new IRSensor. */
  public IRSensor() {}

  public boolean isPickUpComplete() {
    return isPickUpComplete;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Infrared sensor", irSensor.get());
    if (getIRState() && !isPickUpComplete) {
      isPickUpComplete = true;
      //Start timer
      startTime = System.currentTimeMillis();

    }
    if ((System.currentTimeMillis() - startTime)/1000 >= 1) {
      isPickUpComplete = false;
    }
  }
  
  public boolean getIRState() {
    return !irSensor.get();
  }
}
