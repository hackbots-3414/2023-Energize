// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.math;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.lib.math.Conversions;

/** 
 * Mr. Z's example JUnit test for the Conversions utility.  The first example method
 * plays the falconToDegrees() and degreesToFaclon() methods off each other,
 * expecting to end up where it started.  This does not guarantee a Falcon 
 * has 2048 ticks per revolution, but rather our conversion methods are
 * self-consistent.  
 * 
 * These unit tests will run on every WPILib Build Robot Code and are an
 * excellent way to ensure code didn't get broken by a change.
 * 
 */
public class ConversionsTest {

    /**
     * The BeforeEach annotation will trigger this method to be called
     * prior to running any test. It's a good place to instantiate stuff
     * you want to have disappear between tests.
     */
    @BeforeEach
    public void setUp() {
    }

    /**
     * The AfterEach is a place to do perform any clean-up, shut down,
     * etc.
     */
    @AfterEach
    public void tearDown() {
    }

    /**
     * Test methods by convention are prefixed with "test" followed by the
     * name of the method they are testing. If multiple test methods are
     * needed to cover a single source method, that's fine, suffixes can
     * be used to describe the different conditions being tested.
     * 
     * Note that the Test annotation indicates this is a test method.
     */
    @Test
    public void testFalconToDegreesAndDegreesToFalcon() {
        double[] inputs = new double[] { -2049, -2048, -2047, -1, 0, 2048, 2049, 4095, 4096, 4097 };
        double[] gearRatios = new double[] { 10 / 1, 12 / 1, 8 / 1 };
        double resultInDegrees = 0;
        double resultInFalcon = 0;
        for (int inputsCounter = 0; inputsCounter < inputs.length; inputsCounter++) {
            for (int gearRatiosCounter = 0; gearRatiosCounter < gearRatios.length; gearRatiosCounter++) {
                resultInDegrees = Conversions.falconToDegrees(inputs[inputsCounter], gearRatios[gearRatiosCounter]);
                resultInFalcon = Conversions.degreesToFalcon(resultInDegrees, gearRatios[gearRatiosCounter]);
                assertTrue(Math.abs(resultInFalcon - inputs[inputsCounter]) < 0.0001,
                        "Conversion from falcon to degrees to falcon should be within 0.0001 of starting falcon");
            }
        }
    }

    // TODO write the rest of the unit tests for the other methods

}
