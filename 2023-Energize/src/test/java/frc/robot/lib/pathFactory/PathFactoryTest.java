// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.pathFactory;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.math.Conversions;
import frc.lib.pathFactory.PathFactory;

/** Add your docs here. */
public class PathFactoryTest {
    @Test
    public void testPathFactoryConstructor() {
        try {
        assertNotNull(PathFactory.getInstance(), "Got PathFactory instance");            
        } catch (Exception e) {
            e.printStackTrace();
        }

   }

    private Pose2d makePose2d(double x, double y) {
        return new Pose2d(new Translation2d(Conversions.inchesToMeters(x), Conversions.inchesToMeters(y)), new Rotation2d());
    }

    @Test
    public void testInsideCommunity() {
        Pose2d from = makePose2d(600, 50);
        List<Pose2d> path = PathFactory.getInstance().getPath(from, 3);
        List<Pose2d> goal = Arrays.asList(
           from,
           makePose2d(610.77, 50),
           makePose2d(610.77, 64.19)
        );

        assertTrue(goal.size() == path.size());

        for (int i = 0;i < path.size();i ++) {
            assertTrue(path.get(i).equals(goal.get(i)));
        }
    }

    @Test
    public void testOutsideCommunityWithC1() {
        Pose2d from = makePose2d(300, 50);
        List<Pose2d> path = PathFactory.getInstance().getPath(from, 3);
        List<Pose2d> goal = Arrays.asList(
            from,
            makePose2d(438.07, 29.695), // C1
            makePose2d(610.77, 29.695),
            makePose2d(610.77, 64.19)
        );

        assertTrue(goal.size() == path.size(), "The two lists weren't the same size. Goal: " + goal.size() + ", Path recieved: " + path.size());

        for (int i = 0;i < path.size();i ++) {
            assertTrue(path.get(i).equals(goal.get(i)), "The Pose2d at index " + i + " was not the same.  Goal: " + goal.get(i) + "  Path recieved: " + path.get(i));
        }
    }

    @Test
    public void testOutsideCommunityWithC2() {
        Pose2d from = makePose2d(300, 150);
        List<Pose2d> path = PathFactory.getInstance().getPath(from, 3);
        List<Pose2d> goal = Arrays.asList(
            from,
            makePose2d(438.07, 186.335), // C2
            makePose2d(610.77, 186.335),
            makePose2d(610.77, 64.19)
        );

        assertTrue(goal.size() == path.size(), "The two lists weren't the same size. Goal: " + goal.size() + ", Path recieved: " + path.size());

        for (int i = 0;i < path.size();i ++) {
            assertTrue(path.get(i).equals(goal.get(i)), "The Pose2d at index " + i + " was not the same.  Goal: " + goal.get(i) + "  Path recieved: " + path.get(i));
        }
    }

    @Test
    public void testOutsideCommunityWhereC1AndC2AreTheSameDistance1() {
        Pose2d from = makePose2d(300, 108.015);
        List<Pose2d> path = PathFactory.getInstance().getPath(from, 3); // Three is closer to C1, so that is what we should get.
        List<Pose2d> goal = Arrays.asList(
            from,
            makePose2d(438.07, 29.695), // C1, the default.
            makePose2d(610.77, 29.695),
            makePose2d(610.77, 64.19)
        );

        assertTrue(goal.size() == path.size(), "The two lists weren't the same size. Goal: " + goal.size() + ", Path recieved: " + path.size());

        for (int i = 0;i < path.size();i ++) {
            assertTrue(path.get(i).equals(goal.get(i)), "The Pose2d at index " + i + " was not the same.  Goal: " + goal.get(i) + "  Path recieved: " + path.get(i));
        }
    }

    @Test
    public void testOutsideCommunityWhereC1AndC2AreTheSameDistance2() {
        Pose2d from = makePose2d(300, 108.015);
        List<Pose2d> path = PathFactory.getInstance().getPath(from, 9); // Three is closer to C1, so that is what we should get.
        List<Pose2d> goal = Arrays.asList(
            from,
            makePose2d(438.07, 186.335), // C1, the default.
            makePose2d(610.77, 186.335),
            makePose2d(610.77, 196.19)
        );

        assertTrue(goal.size() == path.size(), "The two lists weren't the same size. Goal: " + goal.size() + ", Path recieved: " + path.size());

        for (int i = 0;i < path.size();i ++) {
            assertTrue(path.get(i).equals(goal.get(i)), "The Pose2d at index " + i + " was not the same.  Goal: " + goal.get(i) + "  Path recieved: " + path.get(i));
        }
    }
    /*
     * TODO: Make many tests!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     * What happens when the operator lets go of the button before the path finishes?
     * What happens when the operator hits one button, changes their mind, and hits a different one before finishing the path?
     * What happens when the operator hits 2 buttons at once?
     * What happens when the operator hits a button before leaving the loading zone?
     */

}
