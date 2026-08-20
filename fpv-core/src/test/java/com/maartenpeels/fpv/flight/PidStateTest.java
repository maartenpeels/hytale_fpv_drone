package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class PidStateTest {

    @Nested
    class Priming {

        @Test
        void atRemembersTheRateWithoutRememberingAnyCorrection() {
            PidState primed = PidState.at(6.0);

            assertEquals(0.0, primed.integral(), 1e-12);
            assertEquals(6.0, primed.lastRate(), 1e-12);
        }

        @Test
        void atZeroIsTheSameThingAsZero() {
            assertEquals(PidState.ZERO, PidState.at(0.0));
        }

        @Test
        void atARotatingRateIsNotTheSameThingAsZero() {
            // The distinction the derivative term depends on. RatePidTest is where the size of the
            // difference in torque is measured; this only pins that they are not interchangeable.
            assertNotEquals(PidState.ZERO, PidState.at(6.0));
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsNonFiniteMemoryBecauseItWouldPoisonEveryLaterStep() {
            assertThrows(IllegalArgumentException.class, () -> new PidState(Double.NaN, 0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new PidState(0, Double.POSITIVE_INFINITY));
            assertThrows(IllegalArgumentException.class, () -> PidState.at(Double.NaN));
        }

        @Test
        void acceptsANegativeIntegralBecauseErrorsHaveASign() {
            assertEquals(-0.25, new PidState(-0.25, -3.0).integral(), 1e-12);
        }
    }
}
