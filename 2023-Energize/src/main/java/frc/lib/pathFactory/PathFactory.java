// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.lib.pathFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class PathFactory {
    private static PathFactory me = new PathFactory();

    private PathFactory() {}

    public static PathFactory getInstance() {
        return me;
    }
    private double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    public List<Pose2d> getPath(Pose2d from, int toInt) {

        boolean isRedSide = (DriverStation.getAlliance() == Alliance.Red);

        double x = (isRedSide) ? Constants.PathFactory.redSide.x : Constants.PathFactory.blueSide.x;
        double cx = (isRedSide) ? Constants.PathFactory.redSide.cx : Constants.PathFactory.blueSide.cx;
        double rot = 0;

        Pose2d p1 = new Pose2d(new Translation2d(x, Constants.PathFactory.p1), Rotation2d.fromDegrees(rot));
        Pose2d p2 = new Pose2d(new Translation2d(x, Constants.PathFactory.p2), Rotation2d.fromDegrees(rot));
        Pose2d p3 = new Pose2d(new Translation2d(x, Constants.PathFactory.p3), Rotation2d.fromDegrees(rot));
        Pose2d p4 = new Pose2d(new Translation2d(x, Constants.PathFactory.p4), Rotation2d.fromDegrees(rot));
        Pose2d p5 = new Pose2d(new Translation2d(x, Constants.PathFactory.p5), Rotation2d.fromDegrees(rot));
        Pose2d p6 = new Pose2d(new Translation2d(x, Constants.PathFactory.p6), Rotation2d.fromDegrees(rot));
        Pose2d p7 = new Pose2d(new Translation2d(x, Constants.PathFactory.p7), Rotation2d.fromDegrees(rot));
        Pose2d p8 = new Pose2d(new Translation2d(x, Constants.PathFactory.p8), Rotation2d.fromDegrees(rot));
        Pose2d p9 = new Pose2d(new Translation2d(x, Constants.PathFactory.p9), Rotation2d.fromDegrees(rot));
        Pose2d pC1 = new Pose2d(new Translation2d(cx, Constants.PathFactory.c1), Rotation2d.fromDegrees(rot));
        Pose2d pC2 = new Pose2d(new Translation2d(cx, Constants.PathFactory.c2), Rotation2d.fromDegrees(rot));

        List<Pose2d> scoring_points = Arrays.asList(p1, p2, p3, p4, p5, p6, p7, p8, p9);

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
            //System.out.println("distanceToC1=" + distanceToC1);
            //System.out.println("distanceToC2=" + distanceToC2);

            if (distanceToC1 > distanceToC2) {
                closer = pC2;
            } else {
                closer = pC1;
            }

            result.add(closer);
        } else {
            closer = from;
        }

        Pose2d nextPoint = new Pose2d(new Translation2d(targetX, closer.getY()), Rotation2d.fromDegrees(rot));

        result.add(nextPoint);

        result.add(target);

        return result;
    }
}
