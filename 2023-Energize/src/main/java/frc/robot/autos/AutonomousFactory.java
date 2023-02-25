// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import ch.qos.logback.core.joran.conditional.ThenAction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        BottomObjectOne("Bottom Object One"),
        BottomObjectTwo("Bottom Object Two"),
        BottomObjectThree("Bottom Object Three"),
        BottomObjectFour("Bottom Object Four"),
        BottomObjectFive("Bottom Object Five"),
        BottomObjectSix("Bottom Object Six"),
        BottomObjectSeven("Bottom Object Seven"),
        BottomObjectEight("Bottom Object Eight"),
        TopObjectOne("Top Object One"),
        TopObjectTwo("Top Object Two"),
        TopObjectThree("Top Object Three"),
        TopObjectFour("Top Object Four"),
        TopObjectFive("Top Object Five"),
        TopObjectSix("Top Object Six"),
        TopObjectSeven("Top Object Seven"),
        TopObjectEight("Top Object Eight"),
        AutoBalance("Balance"), 
        TopStart("Top Start"), 
        MidStartTop("Mid Start Top"),
        MidStartLow("Mid Start Low"),
        LowStart("Low Start"),
        DriveOutTop("Drive Out Top"),
        DriveOutLow("Drive Out Low"),
        DriveStraight("DriveStraight");

        public final String value;

        AutonChoice(String value) {
            this.value = value;
        }
    }

    public enum Bays {
        LowLow("Bay Low Low"),
        LowMid("Bay Low Mid"),
        LowHigh("Bay Low High"),
        MidLow("Bay Mid Low"),
        MidMid("Bay Mid Mid"),
        MidHigh("Bay Mid High"),
        HighLow("Bay High Low"),
        HighMid("Bay High Mid"),
        HighHigh("Bay High High");

        public final String value;

        Bays(String value) {
            this.value = value;
        }
    }

    public enum Heights {
        Low,
        Mid,
        High;
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

        PIDController thetaPID = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);

        thetaPID.enableContinuousInput(-180, 180);
        SmartDashboard.putData("Theta PID", thetaPID);

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

    private Command followTrajectoryOnTheFly(PathPoint... pathPoints){
        ArrayList<PathPoint> points = new ArrayList<>();
        for (PathPoint pathPoint : pathPoints) {
            points.add(pathPoint);
        }

        PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared), points);

        return new PPSwerveControllerCommand(
            traj, 
            swerve::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerve::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // Requires this drive subsystem
        );
    }

    private Command followTrajectoryCommand(String pathName, boolean isFirstPath) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   swerve.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 swerve::getPose, // Pose supplier
                 Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 swerve::setModuleStates, // Module states consumer
                 false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 swerve // Requires this drive subsystem
             )
        );
    }

    private Command followTrajectoryWithEventsCommand(String pathName) {
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));        

        return autoBuilder.fullAuto(pathGroup);
    }

    private Command followTrajectoryWithEventsAndOnTheFlyCommand(String pathName) {
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        swerve.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());
        
        return new SequentialCommandGroup(followTrajectoryOnTheFly(resetToVision(), new PathPoint(pathGroup.get(0).getInitialPose().getTranslation(), pathGroup.get(0).getInitialPose().getRotation())), autoBuilder.fullAuto(pathGroup));
    }

    // private Command placeCommand(Heights height) {
    //     if (height == Heights.Low) {
    //         // code to place object low
    //     } else if (height == Heights.Mid) {
    //         // code to place object mid
    //     } else if (height == Heights.High) {
    //         // code to place objects high
    //     }
    //     return null;
    // }

    public Command eventChooser(AutonChoice choice/*, Bays bay, Heights height*/) { // probably should implement a boolean field about whether balance, and add a command
        // return new SequentialCommandGroup(followTrajectoryWithEventsCommand(choice.value), followTrajectoryWithEventsCommand(bay.value), placeCommand(height));
        return followTrajectoryWithEventsCommand(choice.value);
    }

    public Command autobalance() {
        // just so that we can do an auto balance auton if we ever need to
        return new SequentialCommandGroup(followTrajectoryWithEventsCommand(AutonChoice.AutoBalance.value), new PIDBalance(swerve));
    }
}