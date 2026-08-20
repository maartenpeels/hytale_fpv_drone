package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class MotorOutputsTest {

    private static final double TOLERANCE = 1e-12;

    @Nested
    class Thrusts {

        @Test
        void thrustIsTheSquareOfTheCommandSoHoverLandsNearAThirdOfStick() {
            MotorOutputs thrusts = new MotorOutputs(0.5, 0.25, 1.0, 0.0).thrusts();

            assertEquals(0.25, thrusts.frontLeft(), TOLERANCE);
            assertEquals(0.0625, thrusts.frontRight(), TOLERANCE);
            assertEquals(1.0, thrusts.rearLeft(), TOLERANCE);
            assertEquals(0.0, thrusts.rearRight(), TOLERANCE);
        }

        @Test
        void meanOfEqualThrustsIsTheCollectiveFraction() {
            assertEquals(0.25, new MotorOutputs(0.5, 0.5, 0.5, 0.5).thrusts().mean(), TOLERANCE);
            assertEquals(1.0, new MotorOutputs(1, 1, 1, 1).thrusts().mean(), TOLERANCE);
            assertEquals(0.0, MotorOutputs.IDLE.thrusts().mean(), TOLERANCE);
        }
    }

    @Nested
    class Differentials {

        @Test
        void rollIsPositiveWhenTheLeftPairPushesHarderBecauseThatDropsTheRightSide() {
            MotorOutputs leftHeavy = new MotorOutputs(1, 0, 1, 0);

            assertTrue(leftHeavy.rollDifferential() > 0);
            assertEquals(1.0, leftHeavy.rollDifferential(), TOLERANCE);
            assertEquals(0.0, leftHeavy.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, leftHeavy.yawDifferential(), TOLERANCE);
        }

        @Test
        void pitchIsPositiveWhenTheFrontPairPushesHarderBecauseThatDropsTheNose() {
            MotorOutputs frontHeavy = new MotorOutputs(1, 1, 0, 0);

            assertEquals(1.0, frontHeavy.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, frontHeavy.rollDifferential(), TOLERANCE);
            assertEquals(0.0, frontHeavy.yawDifferential(), TOLERANCE);
        }

        @Test
        void yawIsPositiveWhenTheCounterClockwiseDiagonalPushesHarder() {
            // frontLeft and rearRight spin counter-clockwise; the airframe feels the opposite
            // reaction torque, so raising that pair turns the nose right.
            MotorOutputs counterClockwiseHeavy = new MotorOutputs(1, 0, 0, 1);

            assertEquals(1.0, counterClockwiseHeavy.yawDifferential(), TOLERANCE);
            assertEquals(0.0, counterClockwiseHeavy.rollDifferential(), TOLERANCE);
            assertEquals(0.0, counterClockwiseHeavy.pitchDifferential(), TOLERANCE);
        }

        @Test
        void everyDifferentialIsZeroWhenAllFourAreEqual() {
            MotorOutputs balanced = new MotorOutputs(0.4, 0.4, 0.4, 0.4);

            assertEquals(0.0, balanced.rollDifferential(), TOLERANCE);
            assertEquals(0.0, balanced.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, balanced.yawDifferential(), TOLERANCE);
        }
    }
}
