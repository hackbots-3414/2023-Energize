// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class TrajectoryFactory {

    private TrajectoryFactory() {
    }

    // private static Trajectory loadTrajectory(String fileName) {
    //     try {
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(BASE_PATH + fileName);
    //         return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //     } catch (IOException ex) {
    //         DriverStation.reportError("Unable to open trajectory: " + BASE_PATH + fileName, ex.getStackTrace());
    //     }
    //     return null;
    // }

    private static PathPlannerTrajectory loadTrajectory(String fileName, TrajectoryConfig trajectoryConfig) {
        PathPlannerTrajectory path = PathPlanner.loadPath(fileName, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        return path;
    }

    public static Trajectory getPath(String name, Boolean isReversed) {
        TrajectoryConfig config = new TrajectoryConfig(PathweaverConstants.kMaxSpeed, PathweaverConstants.kMaxAcceleration);
        
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(PathweaverConstants.ksVolts, PathweaverConstants.kvVoltSecondsPerMeter, PathweaverConstants.kaVoltSecondsSquaredPerMeter), RobotConstants.kDriveKinematics, 11.5);
        config.addConstraint(new CentripetalAccelerationConstraint(PathweaverConstants.kMaxSpinAcceleration));
        config.setEndVelocity(PathweaverConstants.kMaxEndSpeed);
        config.setKinematics(RobotConstants.kDriveKinematics);
        config.setReversed(isReversed);
        config.addConstraint(autoVoltageConstraint);
        Trajectory trajectory = loadTrajectory(name, config);
        return trajectory;
    }

    // public static Trajectory getPath(String name) {
    //     return getTrajectory(name + ".wpilib.json");
    // }
}