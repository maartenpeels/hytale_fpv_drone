package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class PilotInputSampleTest {

    @Nested
    class AcceptsGarbage {

        @Test
        void takesNonFiniteValuesWithoutThrowingBecauseTheseNumbersCameFromAClient() {
            PilotInputSample sample =
                    new PilotInputSample(
                            Double.NaN,
                            Double.POSITIVE_INFINITY,
                            Double.NaN,
                            Double.NEGATIVE_INFINITY,
                            Double.NaN);

            assertTrue(Double.isNaN(sample.wishX()));
            assertFalse(sample.hasLook());
        }

        @Test
        void takesWildlyOutOfRangeValuesBecauseClampingIsTheMappersJobNotThisTypes() {
            PilotInputSample sample = new PilotInputSample(1e9, -1e9, 1e9, 1e9, 1e9);

            assertEquals(1e9, sample.wishX());
            assertTrue(sample.hasLook());
        }
    }

    @Nested
    class HasLook {

        @Test
        void isTrueOnlyWhenBothLookAnglesArePresent() {
            assertTrue(PilotInputSample.lookRelative(0.0, 0.0, 1.0, 0.5).hasLook());
            assertFalse(new PilotInputSample(0.0, 0.0, 0.0, 1.0, Double.NaN).hasLook());
            assertFalse(new PilotInputSample(0.0, 0.0, 0.0, Double.NaN, 0.5).hasLook());
        }
    }

    @Nested
    class Empty {

        @Test
        void carriesNoWishAndNoLookSoItReadsAsAPacketThatSaidNothing() {
            assertEquals(0.0, PilotInputSample.EMPTY.wishX());
            assertEquals(0.0, PilotInputSample.EMPTY.wishZ());
            assertFalse(PilotInputSample.EMPTY.hasLook());
            assertTrue(Double.isNaN(PilotInputSample.EMPTY.wishFrameYaw()));
        }
    }

    @Nested
    class LookRelative {

        @Test
        void treatsTheLookYawAsTheWishFrameBecauseThatIsHytalesDefaultMovementForceRotation() {
            PilotInputSample sample = PilotInputSample.lookRelative(0.25, -0.5, 1.75, -0.25);

            assertEquals(1.75, sample.wishFrameYaw());
            assertEquals(1.75, sample.lookYaw());
            assertEquals(-0.25, sample.lookPitch());
            assertEquals(0.25, sample.wishX());
            assertEquals(-0.5, sample.wishZ());
        }
    }
}
