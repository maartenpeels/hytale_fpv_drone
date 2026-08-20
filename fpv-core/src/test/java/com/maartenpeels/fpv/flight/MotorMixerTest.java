package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class MotorMixerTest {

    private static final double TOLERANCE = 1e-12;

    private static void assertMotors(
            MotorOutputs actual,
            double frontLeft,
            double frontRight,
            double rearLeft,
            double rearRight) {
        assertEquals(frontLeft, actual.frontLeft(), TOLERANCE, "frontLeft");
        assertEquals(frontRight, actual.frontRight(), TOLERANCE, "frontRight");
        assertEquals(rearLeft, actual.rearLeft(), TOLERANCE, "rearLeft");
        assertEquals(rearRight, actual.rearRight(), TOLERANCE, "rearRight");
    }

    @Nested
    class AxisMapping {

        @Test
        void aNeutralDemandLeavesAllFourMotorsAtTheCollective() {
            assertMotors(MotorMixer.mix(0.5, 0, 0, 0), 0.5, 0.5, 0.5, 0.5);
        }

        @Test
        void rollRaisesTheLeftPairAndLowersTheRight() {
            assertMotors(MotorMixer.mix(0.5, 0.2, 0, 0), 0.7, 0.3, 0.7, 0.3);
        }

        @Test
        void pitchRaisesTheFrontPairAndLowersTheRear() {
            assertMotors(MotorMixer.mix(0.5, 0, 0.2, 0), 0.7, 0.7, 0.3, 0.3);
        }

        @Test
        void yawRaisesTheCounterClockwiseDiagonalAndLowersTheOther() {
            assertMotors(MotorMixer.mix(0.5, 0, 0, 0.2), 0.7, 0.3, 0.3, 0.7);
        }

        @Test
        void axesSuperposeWhenThereIsRoomForAllOfThem() {
            assertMotors(MotorMixer.mix(0.5, 0.1, 0.05, 0.02), 0.67, 0.43, 0.53, 0.37);
        }
    }

    @Nested
    class Desaturation {

        @Test
        void scalesTorquesTogetherSoAPureRollDemandStaysAPureRoll() {
            // Clipping each motor on its own would leave frontLeft and rearLeft unequal to
            // frontRight and rearRight by different amounts -- a phantom pitch or yaw.
            MotorOutputs saturated = MotorMixer.mix(0.5, 1.0, 0, 0);

            assertMotors(saturated, 1.0, 0.0, 1.0, 0.0);
            assertEquals(0.0, saturated.pitchDifferential(), TOLERANCE);
            assertEquals(0.0, saturated.yawDifferential(), TOLERANCE);
        }

        @Test
        void keepsTheRatioBetweenAxesWhenAllThreeAreSaturated() {
            MotorOutputs saturated = MotorMixer.mix(0.5, 1.0, 1.0, 1.0);
            MotorOutputs reference = MotorMixer.mix(0.5, 0.25, 0.25, 0.25);

            // Demanding 1/1/1 can only be met at a quarter scale, which is exactly 0.25/0.25/0.25.
            assertMotors(
                    saturated,
                    reference.frontLeft(),
                    reference.frontRight(),
                    reference.rearLeft(),
                    reference.rearRight());
        }

        @Test
        void pullsTheCollectiveDownRatherThanLoseAttitudeAtFullThrottle() {
            // Air-mode behaviour: a pilot at full throttle who asks for roll gets the roll.
            MotorOutputs atFullThrottle = MotorMixer.mix(1.0, 0.5, 0, 0);

            assertMotors(atFullThrottle, 1.0, 0.0, 1.0, 0.0);
            assertEquals(1.0, atFullThrottle.rollDifferential(), TOLERANCE);
        }

        @Test
        void liftsTheCollectiveRatherThanLeaveNoAuthorityAtClosedThrottle() {
            MotorOutputs atIdle = MotorMixer.mix(0.0, 0.2, 0, 0);

            assertMotors(atIdle, 0.4, 0.0, 0.4, 0.0);
            assertTrue(atIdle.rollDifferential() > 0);
        }

        @Test
        void aFullyDemandingAxisPinsTheCollectiveToMidThrottleWhateverThePilotAsked() {
            // Worth pinning down because it is surprising: once one axis wants the entire command
            // range, the only offset that fits is the middle, so throttle stops having any effect
            // on that motor set at all.
            assertMotors(MotorMixer.mix(0.0, 1.0, 0, 0), 1.0, 0.0, 1.0, 0.0);
            assertMotors(MotorMixer.mix(0.5, 1.0, 0, 0), 1.0, 0.0, 1.0, 0.0);
            assertMotors(MotorMixer.mix(1.0, 1.0, 0, 0), 1.0, 0.0, 1.0, 0.0);
        }

        @Test
        void closedThrottleAndCentredSticksLeavesEveryMotorAtIdle() {
            // Nothing must lift the collective when the pilot is asking for nothing -- this is what
            // makes a dead-stick drop genuinely ballistic.
            assertMotors(MotorMixer.mix(0.0, 0, 0, 0), 0.0, 0.0, 0.0, 0.0);
        }
    }

    @Nested
    class Range {

        @Test
        void neverProducesACommandOutsideZeroToOneAcrossTheWholeDemandSpace() {
            for (int t = 0; t <= 10; t++) {
                for (int r = -4; r <= 4; r++) {
                    for (int p = -4; p <= 4; p++) {
                        for (int y = -4; y <= 4; y++) {
                            MotorOutputs mixed =
                                    MotorMixer.mix(t / 10.0, r / 4.0, p / 4.0, y / 4.0);
                            assertInRange(mixed.frontLeft(), mixed);
                            assertInRange(mixed.frontRight(), mixed);
                            assertInRange(mixed.rearLeft(), mixed);
                            assertInRange(mixed.rearRight(), mixed);
                        }
                    }
                }
            }
        }

        private static void assertInRange(double command, MotorOutputs context) {
            assertTrue(command >= 0.0 && command <= 1.0, command + " out of range in " + context);
        }
    }
}
