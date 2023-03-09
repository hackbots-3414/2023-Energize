// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.pathFactory;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.lib.pathFactory.PathFactory;

/** Add your docs here. */
public class PathFactoryTest {
    @Test
    public void testPathFactoryConstructor(){
        try {
        assertNotNull(PathFactory.getInstance(), "Got PathFactory instance");            
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}
