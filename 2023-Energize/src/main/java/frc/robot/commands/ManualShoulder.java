// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.Intake;


public class ManualShoulder extends CommandBase {
  /** Creates a new ManualShoulder. */

  WPI_TalonFX shoulder = new WPI_TalonFX(Constants.IntakeConstants.shoulderMotorID);

  public ManualShoulder(Intake intake) {

    

    addRequirements(intake);


    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentShoulderPosition = shoulder.getSelectedSensorPosition(); 

    if (currentShoulderPosition == Constants.IntakeConstants.shoulderUpperLimit){
      shoulder.set(0.2); 
    }else{
      shoulder.set(0.0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shoulder.set(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


