package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class PidGainsTest {

    @Nested
    class TimeForm {

        @Test
        void anIntegralTimeIsTheProportionalGainDividedByIt() {
            // The relationship worth pinning: at an integral time of one second the two gains are
            // equal, which is what makes the seconds figure readable as "how long to absorb".
            PidGains gains = PidGains.fromTimes(0.2, 1.0, 0.0);

            assertEquals(0.2, gains.proportional(), 1e-12);
            assertEquals(0.2, gains.integral(), 1e-12);
        }

        @Test
        void halvingTheIntegralTimeDoublesTheIntegralGain() {
            assertEquals(
                    PidGains.fromTimes(0.2, 0.15, 0).integral(),
                    PidGains.fromTimes(0.2, 0.3, 0).integral() * 2,
                    1e-12);
        }

        @Test
        void aDerivativeTimeIsTheProportionalGainMultipliedByIt() {
            assertEquals(0.0016, PidGains.fromTimes(0.2, 0.3, 0.008).derivative(), 1e-12);
        }

        @Test
        void aZeroDerivativeTimeIsAllowedBecauseTheDefaultTuneUsesOne() {
            assertEquals(0.0, PidGains.fromTimes(0.2, 0.3, 0.0).derivative(), 1e-12);
        }

        @Test
        void rejectsANonPositiveIntegralTimeBecauseItDividesTheGain() {
            assertThrows(IllegalArgumentException.class, () -> PidGains.fromTimes(0.2, 0, 0.008));
            assertThrows(
                    IllegalArgumentException.class, () -> PidGains.fromTimes(0.2, -0.1, 0.008));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> PidGains.fromTimes(0.2, Double.NaN, 0.008));
        }

        @Test
        void carriesTheDefaultIntegralLimit() {
            assertEquals(
                    PidGains.DEFAULT_INTEGRAL_LIMIT,
                    PidGains.fromTimes(0.2, 0.3, 0.0).integralLimit(),
                    1e-12);
        }
    }

    @Nested
    class Variants {

        private static final PidGains GAINS = new PidGains(0.2, 1.5, 0.003, 0.3);

        @Test
        void removingTheIntegralTermLeavesEverythingElseAlone() {
            assertEquals(new PidGains(0.2, 0.0, 0.003, 0.3), GAINS.withoutIntegral());
        }

        @Test
        void removingTheDerivativeTermLeavesEverythingElseAlone() {
            assertEquals(new PidGains(0.2, 1.5, 0.0, 0.3), GAINS.withoutDerivative());
        }

        @Test
        void changingTheProportionalGainDoesNotRescaleTheOthers() {
            // Deliberate: these are absolute gains, not a shape plus a magnitude. A tuning UI that
            // wants to scale the whole set has to say so.
            assertEquals(new PidGains(0.8, 1.5, 0.003, 0.3), GAINS.withProportional(0.8));
        }

        @Test
        void changingTheIntegralLimitDoesNotTouchTheGains() {
            assertEquals(new PidGains(0.2, 1.5, 0.003, 0.0), GAINS.withIntegralLimit(0.0));
        }
    }

    @Nested
    class Validation {

        @Test
        void acceptsZeroGainsBecausePOnlyAndPiOnlyAreRealTunes() {
            assertEquals(0.0, new PidGains(0.2, 0, 0, 0.3).integral(), 1e-12);
            assertEquals(0.0, new PidGains(0, 0, 0, 0).proportional(), 1e-12);
        }

        @Test
        void rejectsNegativeGainsBecauseTheyInvertTheFeedbackLoop() {
            assertThrows(IllegalArgumentException.class, () -> new PidGains(-0.1, 1, 0, 0.3));
            assertThrows(IllegalArgumentException.class, () -> new PidGains(0.2, -1, 0, 0.3));
            assertThrows(IllegalArgumentException.class, () -> new PidGains(0.2, 1, -0.1, 0.3));
            assertThrows(IllegalArgumentException.class, () -> new PidGains(0.2, 1, 0, -0.3));
        }

        @Test
        void rejectsNonFiniteGains() {
            assertThrows(
                    IllegalArgumentException.class, () -> new PidGains(Double.NaN, 1, 0, 0.3));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new PidGains(0.2, Double.POSITIVE_INFINITY, 0, 0.3));
        }

        @Test
        void rejectsAnIntegralLimitAboveFullOutputAuthority() {
            // The limit is a fraction of the output range, so anything above 1 could never bind and
            // would silently leave the accumulator unbounded in the cases the freeze does not catch.
            assertThrows(IllegalArgumentException.class, () -> new PidGains(0.2, 1, 0, 1.5));
            assertEquals(1.0, new PidGains(0.2, 1, 0, 1.0).integralLimit(), 1e-12);
        }
    }
}
