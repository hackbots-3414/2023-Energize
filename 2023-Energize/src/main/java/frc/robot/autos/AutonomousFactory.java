// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutonomousFactory {
    private static final AutonomousFactory me = new AutonomousFactory();

    private static Swerve swerve;

    private AutonomousFactory() {}

    public static AutonomousFactory getInstance(Swerve m_swerve) {
        swerve = m_swerve;
        return me;
    }

    public SwerveControllerCommand followTrajectoryCommand(String pathName){
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Swerve.swerveKinematics);        

        PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        List<EventMarker> markers = traj.getMarkers();
        ArrayList<Translation2d> waypoints = new ArrayList<>();

        for (EventMarker marker : markers) {
            waypoints.add(marker.positionMeters);
        }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                traj.getInitialHolonomicPose(),
                waypoints,
                traj.getEndState().poseMeters,
                config);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommandProxy swerveControllerCommand = new SwerveControllerCommandProxy(
                trajectory, swerve::getPose, Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController, swerve::setModuleStates, swerve);
            
        return swerveControllerCommand;
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

    public SequentialCommandGroup testAuto() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        group.addCommands(new SequentialCommandGroup(followTrajectoryCommand("testPath", true)));
        return group;
    }

    public class SwerveControllerCommandProxy extends SwerveControllerCommand{

        public SwerveControllerCommandProxy(Trajectory trajectory,
        Supplier<Pose2d> pose,
        SwerveDriveKinematics kinematics,
        PIDController xController,
        PIDController yController,
        ProfiledPIDController thetaController,
        Consumer<SwerveModuleState[]> outputModuleStates,
        Subsystem... requirements) {
            super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements);
        }

        @Override
        public void initialize() {
            super.initialize();
        }

        @Override
        public void execute() {
            //swerve.updateOdometry();
            super.execute();
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
        }
        
        @Override
        public boolean isFinished() {
            return super.isFinished();
        }
    }
}