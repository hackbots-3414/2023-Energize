// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PIDBalance;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class AutonomousFactory {

    public int choice;

    public enum AutonChoice {
        // BottomObjectOne(),
        // BottomObjectTwo(),
        // BottomObjectThree(),
        // BottomObjectFour(),
        // BottomObjectFive(),
        // BottomObjectSix(),
        // BottomObjectSeven(),
        // BottomObjectEight(),
        // TopObjectOne(),
        // TopObjectTwo(),
        // TopObjectThree(),
        // TopObjectFour(),
        // TopObjectFive(),
        // TopObjectSix(),
        // TopObjectSeven(),
        // TopObjectEight(),
        AutoBalance("Balance"), 
        // TopStart(), 
        // MidStartTop(),
        // MidStartLow(),
        // LowStart(),
        DriveOutTop("Drive Out Top"),
        DriveOutLow("Drive Out Low");

        public final String value;

        AutonChoice(String value) {
            this.value = value;
        }
    }

    private static final AutonomousFactory me = new AutonomousFactory();

    private static Swerve swerve;

    private static HashMap<String, Command> eventMap = new HashMap<>();

    private static SwerveAutoBuilder autoBuilder;

    private AutonomousFactory() {}

    public static AutonomousFactory getInstance(Swerve m_swerve, Intake intake, Wrist wrist, Shoulder shoulder) {
        swerve = m_swerve;
        // eventMap.put("startIntake", new SequentialCommandGroup(new IntakeAuto(wrist, shoulder, 0), new InstantCommand(() -> intake.spinHand(Constants.IntakeConstants.intakeSpeedPercent))));
        // eventMap.put("stopIntake", new SequentialCommandGroup(new IntakeAuto(wrist, shoulder, 0), new InstantCommand(() -> intake.spinHand(0))));
        eventMap.put("startIntake", new PrintCommand("Start Intake Event Worked!!!"));
        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, 
            swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), 
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);

        return me;
    }

    public PathPoint resetToVision() {
        swerve.updateOdometry();
        Pose2d pose = swerve.getPose();
        return new PathPoint(pose.getTranslation(), pose.getRotation());
    }

    // private Command followTrajectoryOnTheFly(PathPoint... pathPoints){
    //     ArrayList<PathPoint> points = new ArrayList<>();
    //     for (PathPoint pathPoint : pathPoints) {
    //         points.add(pathPoint);
    //     }

    //     PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared), points);

    //     return new PPSwerveControllerCommand(
    //         traj, 
    //         swerve::getPose, // Pose supplier
    //         Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    //         new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //         new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
    //         new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //         swerve::setModuleStates, // Module states consumer
    //         true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //         swerve // Requires this drive subsystem
    //     );
    // }

    // private Command followTrajectoryCommand(String pathName, boolean isFirstPath) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    //     return new SequentialCommandGroup(
    //          new InstantCommand(() -> {
    //            // Reset odometry for the first path you run during auto
    //            if(isFirstPath){
    //                swerve.resetOdometry(traj.getInitialHolonomicPose());
    //            }
    //          }),
    //          new PPSwerveControllerCommand(
    //              traj, 
    //              swerve::getPose, // Pose supplier
    //              Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    //              new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //              new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
    //              new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //              swerve::setModuleStates, // Module states consumer
    //              false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //              swerve // Requires this drive subsystem
    //          )
    //     );
    // }

    // private Command followTrajectoryWithEventsAndOnTheFlyCommand(String pathName) {
    //     ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    //     return new SequentialCommandGroup(followTrajectoryOnTheFly(resetToVision(), new PathPoint(pathGroup.get(0).getInitialPose().getTranslation(), pathGroup.get(0).getInitialPose().getRotation())), autoBuilder.fullAuto(pathGroup));
    // }

    private Command followTrajectoryWithEventsCommand(String pathName) {
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command eventChooser(AutonChoice choice) {
        // return followTrajectoryWithEventsCommand("Drive Out Top");
        return followTrajectoryWithEventsCommand(choice.value);
    }

    public Command autobalance() {
        return new SequentialCommandGroup(followTrajectoryWithEventsCommand(AutonChoice.AutoBalance.value), new PIDBalance(swerve));
    }
}