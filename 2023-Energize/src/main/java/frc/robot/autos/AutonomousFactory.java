// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Wait;
import frc.robot.commands.AutoArm;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PIDBalance;
import frc.robot.commands.ejectCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class AutonomousFactory {

    public int choice;

    public enum AutonChoice {
        Balance("Mid Balance"),
        Left("Left"),
        Right("Right"),
        Nothing("Nothing"),
        WallHigh("Wall High"),
        BarrierHigh("Barrier High"),
        BalanceHigh("Balance High"),
        Test("Test"),
        BarrierHighTwoObject("Barrier High Two Object"),
        WallHighTwoObject("Wall High Two Object");

        public final String value;

        AutonChoice(String value) {
            this.value = value;
        }
    }

    private static final AutonomousFactory me = new AutonomousFactory();

    private static Swerve swerve;

    private static HashMap<String, Command> eventMap = new HashMap<>();

    private AutonomousFactory() {}

    public static void setup(Swerve m_swerve, Intake m_intake, Wrist m_wrist, Shoulder m_shoulder) {
        swerve = m_swerve;

        eventMap.put("Eject", new ejectCommand(m_intake).withTimeout(0.2));
        eventMap.put("Intake", new IntakeCommand(m_intake).withTimeout(3));
        eventMap.put("Mid", new AutoArm(m_shoulder, m_wrist, 3));
        eventMap.put("High", new AutoArm(m_shoulder, m_wrist, 4));
        eventMap.put("PickUp", new AutoArm(m_shoulder, m_wrist, 1));
        eventMap.put("Stow", new AutoArm(m_shoulder, m_wrist, 0));
        eventMap.put("Balance", new PIDBalance(swerve, true));
        eventMap.put("Wait", new Wait(1.0));


        // generate a configuration:
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, Constants.AutoConstants.kDThetaController),
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kDriveBaseRadius,
            new ReplanningConfig(true, true)
            );

        AutoBuilder.configureHolonomic(
            swerve::getPose, 
            swerve::resetOdometry,
            swerve::getCurrentChassisSpeeds,
            swerve::setCurrentChassisSpeeds,
            config,
            swerve);
    }

    private Command followTrajectoryWithEventsCommand(String pathName) {
        return new PathPlannerAuto(pathName);
        
    }

    public Command eventChooser(AutonChoice choice) {
        return followTrajectoryWithEventsCommand(choice.value);
    }
}