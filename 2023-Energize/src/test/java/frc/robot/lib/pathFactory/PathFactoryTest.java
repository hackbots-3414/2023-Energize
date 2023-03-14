// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.pathFactory;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

}
