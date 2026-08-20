package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class MotorThrustsTest {

    private static final double TOLERANCE = 1e-12;

    @Nested
    class Collective {

        @Test
        void isTheMeanOfTheFourThrusts() {
            assertEquals(0.25, new MotorThrusts(0.25, 0.25, 0.25, 0.25).collective(), TOLERANCE);
            assertEquals(0.5, new MotorThrusts(1, 1, 0, 0).collective(), TOLERANCE);
            assertEquals(0.0, MotorThrusts.NONE.collective(), TOLERANCE);
        }
    }

    @Nested
    class Differentials {

        // Each differential is the torque about one axis. The signs are what tie the mixer to
        // BodyRates' pilot convention, so they are asserted as directions, not just magnitudes.

        @Test
        void rollIsPositiveWhenTheLeftPairPushesHarderBecauseThatDropsTheRightSide() {
            MotorThrusts leftHeavy = new MotorThrusts(1, 0, 1, 0);

            assertTrue(leftHeavy.rollDifferential() > 0);
            assertEquals(1.0, leftHeavy.rollDifferential(), TOLERANCE);
            assertEquals(0.0, leftHeavy.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, leftHeavy.yawDifferential(), TOLERANCE);
        }

        @Test
        void pitchIsPositiveWhenTheFrontPairPushesHarderBecauseThatDropsTheNose() {
            MotorThrusts frontHeavy = new MotorThrusts(1, 1, 0, 0);

            assertEquals(1.0, frontHeavy.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, frontHeavy.rollDifferential(), TOLERANCE);
            assertEquals(0.0, frontHeavy.yawDifferential(), TOLERANCE);
        }

        @Test
        void yawIsPositiveWhenTheCounterClockwiseDiagonalPushesHarder() {
            // frontLeft and rearRight spin counter-clockwise; the airframe feels the opposite
            // reaction torque, so raising that pair turns the nose right.
            MotorThrusts counterClockwiseHeavy = new MotorThrusts(1, 0, 0, 1);

            assertEquals(1.0, counterClockwiseHeavy.yawDifferential(), TOLERANCE);
            assertEquals(0.0, counterClockwiseHeavy.rollDifferential(), TOLERANCE);
            assertEquals(0.0, counterClockwiseHeavy.pitchDifferential(), TOLERANCE);
        }

        @Test
        void everyDifferentialIsZeroWhenAllFourAreEqual() {
            MotorThrusts balanced = new MotorThrusts(0.4, 0.4, 0.4, 0.4);

            assertEquals(0.0, balanced.rollDifferential(), TOLERANCE);
            assertEquals(0.0, balanced.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, balanced.yawDifferential(), TOLERANCE);
        }

        @Test
        void reverseTheImbalanceAndTheDifferentialReverses() {
            assertEquals(
                    -new MotorThrusts(1, 0, 1, 0).rollDifferential(),
                    new MotorThrusts(0, 1, 0, 1).rollDifferential(),
                    TOLERANCE);
        }
    }
}
