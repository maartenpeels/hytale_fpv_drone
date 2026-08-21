package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class PilotInputMappingTest {

    @Nested
    class Validation {

        @Test
        void rejectsAZeroFullScaleBecauseEveryOneOfThemIsADivisor() {
            assertThrows(
                    IllegalArgumentException.class, () -> new PilotInputMapping(0.0, 1.0, 1.0));
            assertThrows(
                    IllegalArgumentException.class, () -> new PilotInputMapping(1.0, 0.0, 1.0));
            assertThrows(
                    IllegalArgumentException.class, () -> new PilotInputMapping(1.0, 1.0, 0.0));
        }

        @Test
        void rejectsNegativeFullScaleBecauseItWouldSilentlyInvertAnAxis() {
            assertThrows(
                    IllegalArgumentException.class, () -> new PilotInputMapping(-1.0, 1.0, 1.0));
        }

        @Test
        void rejectsNonFiniteFullScale() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new PilotInputMapping(Double.NaN, 1.0, 1.0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new PilotInputMapping(1.0, Double.POSITIVE_INFINITY, 1.0));
        }
    }

    @Nested
    class Default {

        @Test
        void assumesAUnitWishVectorSoAnUnnormalisedOneShowsUpAsASaturatedAxis() {
            assertEquals(1.0, PilotInputMapping.DEFAULT.wishFullScale());
        }

        @Test
        void needsOneViewRevolutionPerSecondForFullStickOnBothLookAxes() {
            assertEquals(2.0 * Math.PI, PilotInputMapping.DEFAULT.rollLookRateFullScale());
            assertEquals(2.0 * Math.PI, PilotInputMapping.DEFAULT.pitchLookRateFullScale());
        }
    }
}
