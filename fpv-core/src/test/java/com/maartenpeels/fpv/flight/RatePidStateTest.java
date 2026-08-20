package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class RatePidStateTest {

    @Nested
    class Priming {

        @Test
        void atPrimesEachAxisWithItsOwnRate() {
            RatePidState primed = RatePidState.at(new BodyRates(1, -2, 3));

            assertEquals(1.0, primed.roll().lastRate(), 1e-12);
            assertEquals(-2.0, primed.pitch().lastRate(), 1e-12);
            assertEquals(3.0, primed.yaw().lastRate(), 1e-12);
        }

        @Test
        void atCarriesNoAccumulatedCorrectionOnAnyAxis() {
            RatePidState primed = RatePidState.at(new BodyRates(1, -2, 3));

            assertEquals(0.0, primed.roll().integral(), 1e-12);
            assertEquals(0.0, primed.pitch().integral(), 1e-12);
            assertEquals(0.0, primed.yaw().integral(), 1e-12);
        }

        @Test
        void zeroIsWhatAtProducesForAStationaryDrone() {
            assertEquals(RatePidState.ZERO, RatePidState.at(BodyRates.ZERO));
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsAMissingAxis() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new RatePidState(null, PidState.ZERO, PidState.ZERO));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new RatePidState(PidState.ZERO, null, PidState.ZERO));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new RatePidState(PidState.ZERO, PidState.ZERO, null));
        }

        @Test
        void rejectsNonFiniteOrMissingRates() {
            assertThrows(IllegalArgumentException.class, () -> RatePidState.at(null));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> RatePidState.at(new BodyRates(Double.NaN, 0, 0)));
        }
    }
}
