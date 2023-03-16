// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.pathFactory.PathFactory;
import frc.robot.Constants;
import frc.robot.commands.AutoArm;
import frc.robot.commands.PIDBalance;
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
        Nothing("Nothing");

        public final String value;

        AutonChoice(String value) {
            this.value = value;
        }
    }

    public enum Bays {
        One(1),
        Two(2),
        Three(3),
        Four(4),
        Five(5),
        Six(6),
        Seven(7),
        Eight(8),
        Nine(9);

        public final int value;

        Bays(int value) {
            this.value = value;
        }
    }

    private static final AutonomousFactory me = new AutonomousFactory();

    private static Swerve swerve;
    private static Intake intake;

    private static HashMap<String, Command> eventMap = new HashMap<>();

    private static SwerveAutoBuilder autoBuilder;

    private AutonomousFactory() {}

    public static AutonomousFactory getInstance(Swerve m_swerve, Intake m_intake, Wrist m_wrist, Shoulder m_shoulder) {
        swerve = m_swerve;
        intake = m_intake;
        // eventMap.put("ShootHigh", new SequentialCommandGroup(new IntakeCommand(wrist, shoulder, 0), new InstantCommand(() -> intake.spinHand(Constants.IntakeConstants.intakeSpeedPercent))));
        // eventMap.put("IntakeEnd", new SequentialCommandGroup(new IntakeAuto(wrist, shoulder, 0), new InstantCommand(() -> intake.spinHand(0))));
        eventMap.put("Eject", new SequentialCommandGroup(new InstantCommand(() -> intake.set(Constants.IntakeConstants.ejectSpeedAutonPercent)), new InstantCommand(() -> Timer.delay(0.1)), new InstantCommand(() -> intake.set(0))));
        eventMap.put("Mid", new AutoArm(m_shoulder, m_wrist, 3));
        eventMap.put("High", new AutoArm(m_shoulder, m_wrist, 4));
        eventMap.put("Stow", new AutoArm(m_shoulder, m_wrist, 0));
        eventMap.put("Balance", new PIDBalance(swerve, true));

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
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // Requires this drive subsystem
        );
    }

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

    public Command eventChooser(AutonChoice choice) {
        return followTrajectoryWithEventsCommand(choice.value);
    }

    private Command goToBayCommand(int bay) {
        Pose2d from = swerve.getPose();
        List<Pose2d> pose2d_points =  PathFactory.getInstance().getPath(from, bay);
        List<PathPoint> points = new ArrayList<PathPoint>();
        for (int i = 0;i < pose2d_points.size();i ++) {
            points.add(new PathPoint(pose2d_points.get(i).getTranslation(), pose2d_points.get(i).getRotation()));
        }
        if (points.size() == 3) {
            return followTrajectoryOnTheFly(points.get(0), points.get(1), points.get(2));
        } else {
            // Otherwise it will be four.
            return followTrajectoryOnTheFly(points.get(0), points.get(1), points.get(2), points.get(3));
        }
    }

    public Command bayChooser(Bays bay) {
        return goToBayCommand(bay.value);
    }
}