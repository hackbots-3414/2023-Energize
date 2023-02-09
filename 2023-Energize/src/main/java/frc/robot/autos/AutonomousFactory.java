// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutonomousFactory {
    private static final AutonomousFactory me = new AutonomousFactory();

    private static Swerve swerve;

    private AutonomousFactory() {
    }

    public static AutonomousFactory getInstance(Swerve m_swerve) {
        swerve = m_swerve;
        return me;
    }

    // CREATING COMMANDS

    
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
                 new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // Y controller (usually the same values as X controller)
                 new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
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

    public SequentialCommandGroup move1CM() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        group.addCommands(new SequentialCommandGroup(followTrajectoryCommand("move1CM", true)));
        return group;
    }

    public class RamseteCommandProxy extends RamseteCommand {
        private Trajectory trajectory;

        public RamseteCommandProxy(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController, PIDController rightController, BiConsumer<Double, Double> outputVolts, Subsystem... requirements) {
            super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements);
            this.trajectory = trajectory;
        }

        @Override
        public void initialize() {
            swerve.resetOdometry(trajectory.getInitialPose());
            super.initialize();
        }

        @Override
        public void execute() {
            super.execute();
        }

        @Override
        public void end(boolean interrupted) {
            swerve.drive(new Translation2d(0, 0), 0, true, true);
            super.end(interrupted);
        }
    }
}