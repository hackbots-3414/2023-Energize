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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutonomousFactory {
    private static final AutonomousFactory me = new AutonomousFactory();

    private static Swerve swerve;

    private static PathPoint object1 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint object2 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint object3 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint object4 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());

    private static PathPoint object5 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint object6 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint object7 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint object8 = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());

    private static PathPoint leave = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());

    private static PathPoint balanceLeft = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint balanceMiddle = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    private static PathPoint balanceRight = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());




    private AutonomousFactory() {}

    public static AutonomousFactory getInstance(Swerve m_swerve) {
        swerve = m_swerve;
        return me;
    }

    public PathPoint resetToVision() {
        swerve.updateOdometry();
        Pose2d pose = swerve.getPose();
        return new PathPoint(pose.getTranslation(), pose.getRotation());
    }

    public Command followTrajectoryOnTheFly(PathPoint... pathPoints){
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
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 swerve // Requires this drive subsystem
             )
        );
    }

    private Command followTrajectoryWithEventsCommand(String pathName) {
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("intakeDown", new PrintCommand("intakeDown"));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, 
            swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), 
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);

        return autoBuilder.fullAuto(pathGroup);
    }

    public Command driveStraight() {
        return followTrajectoryCommand("DriveStraight", true);
    }
}