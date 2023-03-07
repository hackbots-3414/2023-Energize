// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pathFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.TreeMap;

//import javax.swing.text.Position;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;
import frc.robot.SwerveModule;
//import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Swerve;;

/** Add your docs here. */
public class PathFactory {
    private static PathFactory me = new PathFactory();
    private Pose2d pose = new Pose2d();
    private Translation2d translation = new Translation2d();

    // I made these public because they should be used when using the getPath() method.
    // TODO: Put in values for the positions of the things. Also ensure that you switch the side depending on the side of the field we're on.

    private boolean isRedSide = true;
    private double x = (isRedSide) ? Constants.PathFactory.redSide.x : Constants.PathFactory.blueSide.x;

    private double abx = (isRedSide) ? Constants.PathFactory.redSide.abx : Constants.PathFactory.blueSide.abx;
    private double cx = (isRedSide) ? Constants.PathFactory.redSide.cx : Constants.PathFactory.blueSide.cx;

    public Pose2d p1 = new Pose2d(new Translation2d(x, Constants.PathFactory.p1), new Rotation2d());
    public Pose2d p2 = new Pose2d(new Translation2d(x, Constants.PathFactory.p2), new Rotation2d());
    public Pose2d p3 = new Pose2d(new Translation2d(x, Constants.PathFactory.p3), new Rotation2d());
    public Pose2d p4 = new Pose2d(new Translation2d(x, Constants.PathFactory.p4), new Rotation2d());
    public Pose2d p5 = new Pose2d(new Translation2d(x, Constants.PathFactory.p5), new Rotation2d());
    public Pose2d p6 = new Pose2d(new Translation2d(x, Constants.PathFactory.p6), new Rotation2d());
    public Pose2d p7 = new Pose2d(new Translation2d(x, Constants.PathFactory.p7), new Rotation2d());
    public Pose2d p8 = new Pose2d(new Translation2d(x, Constants.PathFactory.p8), new Rotation2d());
    public Pose2d p9 = new Pose2d(new Translation2d(x, Constants.PathFactory.p9), new Rotation2d());

    public Pose2d pA = new Pose2d(new Translation2d(abx, Constants.PathFactory.a), new Rotation2d());
    public Pose2d pB = new Pose2d(new Translation2d(abx, Constants.PathFactory.b), new Rotation2d());
    public Pose2d pC1 = new Pose2d(new Translation2d(/*PUT VALUES HERE*/cx, Constants.PathFactory.c1), new Rotation2d());
    public Pose2d pC2 = new Pose2d(new Translation2d(/*PUT VALUES HERE*/cx, Constants.PathFactory.c2), new Rotation2d());

    private List<Pose2d> poses = Arrays.asList(p1, p2, p3, p4, p5, p6, p7, p8, p9, pA, pB, pC1, pC2);

    private List<Pose2d> pA_path = Arrays.asList(pC2, p9, p8, p7, p6, p5, p4, p3, p2, p1);
    private List<Pose2d> pB_path = Arrays.asList(pC1, p1, p2, p3, p4, p5, p6, p7, p8, p9);
    private List<Pose2d> pC1_path = Arrays.asList(p1, p2, p3, p4, p5, p6, p7, p8, p9);
    private List<Pose2d> pC2_path = Arrays.asList(p9, p8, p7, p6, p5, p4, p3, p2, p1);

    private TreeMap<Pose2d, List<Pose2d>> paths = new TreeMap<Pose2d, List<Pose2d>>();

    private PathFactory() {
        paths.put(pA, pA_path);
        paths.put(pB, pB_path);
        paths.put(pC1, pC1_path);
        paths.put(pC2, pC2_path);
    }

    public static PathFactory getInstance() {
        return me;
    }

    private double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
        // Pose2d relative = a.relativeTo(b);
        // double distance = Math.sqrt(
        //     Math.pow(relative.getX(), 2)
        //     +
        //     Math.pow(relative.getY(), 2)
        // );
        // return distance;
    }

    public List<Pose2d> getPath(Pose2d from, int toInt) {

        Pose2d to = pA_path.get(toInt); // figure out which scoring point to go to

        

        // add all of the points in between:
        /*
            a. loop through all of the points in the point list
            b. find the point that would bring us closest to the destination
            c. Find that point's path.
            d. Add each point in the path.
        */

        Pose2d best_start_pose = poses.get(0); // It has to be initialized with something

        double currentDistanceToTarget = getDistance(from, to);

        double currentBestDistanceToNextWaypoint = getDistance(from, best_start_pose);
        
        for (int i = 0;i < poses.size();i ++) {
            /*
             * Find the point that is closest to the robot currently and DOES NOT take us farther away (using currentDistance)
             */

            Pose2d thisPose = poses.get(i);

            if (getDistance(from, thisPose) < currentBestDistanceToNextWaypoint) {
                if (getDistance(thisPose, to) < currentDistanceToTarget) {
                    best_start_pose = thisPose;
                    currentBestDistanceToNextWaypoint = getDistance(from, thisPose);
                }
            }

            
        }

        boolean outside = false;

        if (best_start_pose == pA || best_start_pose == pB || best_start_pose == pC1 || best_start_pose == pC2) {
            outside = true;
        }

        if (!outside) {
            return Arrays.asList(to, from);
        }

        List<Pose2d> path = paths.get(best_start_pose);

        ArrayList<Pose2d> results = new ArrayList<Pose2d>();
        results.add(from);

        for (int i = 0;i < path.size();i++) {
            if (path.get(i) != to) {
                results.add(path.get(i));
            } else {
                break;
            }
        }

        results.add(to);

        return results;
    }

}
