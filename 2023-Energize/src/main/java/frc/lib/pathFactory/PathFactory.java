// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.lib.pathFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;

/** Add your docs here. */
public class PathFactory {
    private static PathFactory me = new PathFactory();

    private boolean isRedSide = (DriverStation.getAlliance() == Alliance.Red); // If driver station is NOT open, it will evaluate to true for some reason.

    private double x = (isRedSide) ? Constants.PathFactory.redSide.x : Constants.PathFactory.blueSide.x;

    private double cx = (isRedSide) ? Constants.PathFactory.redSide.cx : Constants.PathFactory.blueSide.cx;

    private Pose2d p1 = new Pose2d(new Translation2d(x, Constants.PathFactory.p1), new Rotation2d());
    private Pose2d p2 = new Pose2d(new Translation2d(x, Constants.PathFactory.p2), new Rotation2d());
    private Pose2d p3 = new Pose2d(new Translation2d(x, Constants.PathFactory.p3), new Rotation2d());
    private Pose2d p4 = new Pose2d(new Translation2d(x, Constants.PathFactory.p4), new Rotation2d());
    private Pose2d p5 = new Pose2d(new Translation2d(x, Constants.PathFactory.p5), new Rotation2d());
    private Pose2d p6 = new Pose2d(new Translation2d(x, Constants.PathFactory.p6), new Rotation2d());
    private Pose2d p7 = new Pose2d(new Translation2d(x, Constants.PathFactory.p7), new Rotation2d());
    private Pose2d p8 = new Pose2d(new Translation2d(x, Constants.PathFactory.p8), new Rotation2d());
    private Pose2d p9 = new Pose2d(new Translation2d(x, Constants.PathFactory.p9), new Rotation2d());

    private Pose2d pC1 = new Pose2d(new Translation2d(cx, Constants.PathFactory.c1), new Rotation2d());
    private Pose2d pC2 = new Pose2d(new Translation2d(cx, Constants.PathFactory.c2), new Rotation2d());

    private List<Pose2d> scoring_points = Arrays.asList(p1, p2, p3, p4, p5, p6, p7, p8, p9);
  
    int pA_INDEX = 0;
    int pB_INDEX = 1;
    int pC1_INDEX = 2;
    int pC2_INDEX = 3;

    private PathFactory() {}

    public static PathFactory getInstance() {
        return me;
    }
    private double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    public List<Pose2d> getPath(Pose2d from, int toInt) {
        Pose2d target = scoring_points.get(toInt - 1); // figure out which scoring point to go to (this does that)

        double targetX = target.getX();

        double outsideBarrier = pC1.getX();

        boolean outside = false;

        if (isRedSide && from.getX() < outsideBarrier) {
            outside = true;
        } else if (!isRedSide && from.getX() > outsideBarrier) {
            outside = true;
        }

        List<Pose2d> result = new ArrayList<Pose2d>();

        result.add(from);
        Pose2d closer;

        if (outside) {
            // which is closer: C1 or C2:
            double distanceToC1 = getDistance(from, pC1) + getDistance(pC1, target);
            double distanceToC2 = getDistance(from, pC2) + getDistance(pC2, target);

            if (distanceToC1 > distanceToC2) {
                closer = pC2;
            } else {
                closer = pC1;
            }

            result.add(closer);
        } else {
            closer = from;
        }

        Pose2d nextPoint = new Pose2d(new Translation2d(targetX, closer.getY()), new Rotation2d());

        result.add(nextPoint);

        result.add(target);

        return result;
    }
}
