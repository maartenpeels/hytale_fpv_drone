package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class RateCurveTest {

    private static final RateCurve DEFAULT = RateProfile.DEFAULT.roll();

    /** Expo values spanning the range, including both ends. */
    private static final double[] EXPOS = {0.0, 0.1, 0.25, 0.54, 0.75, 0.9, 1.0};

    /** Tunes a pilot might plausibly type, in degrees per second. */
    private static final double[][] TUNES = {
        {200, 800}, {100, 400}, {50, 1200}, {1, 2000}, {670, 670}, {360, 361}, {10, 3000}
    };

    @Nested
    class Endpoints {

        @Test
        void centredStickDemandsNoRotation() {
            for (double expo : EXPOS) {
                for (double[] tune : TUNES) {
                    RateCurve curve = RateCurve.fromDegrees(tune[0], tune[1], expo);

                    assertEquals(0.0, curve.rateFor(0.0), () -> "expo " + expo + " tune " + tune[1]);
                }
            }
        }

        /**
         * Exactly, not approximately — {@code maxRate} is the number a tuning UI shows and a pilot
         * reads off the curve, so it has to be the number the curve actually reaches.
         * {@link RateCurve#rateFor}'s grouping exists for this test.
         */
        @Test
        void fullStickDemandsExactlyTheConfiguredMaxRate() {
            for (double expo : EXPOS) {
                for (double[] tune : TUNES) {
                    RateCurve curve = RateCurve.fromDegrees(tune[0], tune[1], expo);

                    assertEquals(curve.maxRate(), curve.rateFor(1.0));
                    assertEquals(-curve.maxRate(), curve.rateFor(-1.0));
                }
            }
        }

        /**
         * The endpoint has to survive the arrangement for every tune, not just the tidy ones. The
         * grouping this pins down was chosen against this sweep: the algebraically identical
         * {@code Cx + (M − C)s} misses by one ULP on about 5 % of these.
         */
        @Test
        void fullStickIsExactAcrossAWholeDegreeSweepOfTunes() {
            for (int centre = 1; centre <= 400; centre += 3) {
                for (int max = centre; max <= 2000; max += 17) {
                    RateCurve curve = RateCurve.fromDegrees(centre, max, 0.54);
                    String tune = centre + " -> " + max + " deg/s";

                    assertEquals(curve.maxRate(), curve.rateFor(1.0), tune);
                    assertEquals(-curve.maxRate(), curve.rateFor(-1.0), tune);
                }
            }
        }
    }

    @Nested
    class Shape {

        @Test
        void isMonotonicAcrossTheFullStickRange() {
            for (double expo : EXPOS) {
                for (double[] tune : TUNES) {
                    RateCurve curve = RateCurve.fromDegrees(tune[0], tune[1], expo);

                    double previous = curve.rateFor(-1.0);
                    for (int i = 1; i <= 4000; i++) {
                        double stick = -1.0 + 2.0 * i / 4000.0;
                        double rate = curve.rateFor(stick);

                        assertTrue(
                                rate > previous,
                                "rate fell from " + previous + " to " + rate + " at stick " + stick
                                        + " on expo " + expo);
                        previous = rate;
                    }
                }
            }
        }

        /** #14's rate loop and the mixer are tuned against a demand that cannot exceed this. */
        @Test
        void neverDemandsMoreThanTheMaxRate() {
            for (double expo : EXPOS) {
                for (double[] tune : TUNES) {
                    RateCurve curve = RateCurve.fromDegrees(tune[0], tune[1], expo);

                    for (int i = 0; i <= 2000; i++) {
                        double stick = -1.0 + 2.0 * i / 2000.0;

                        assertTrue(
                                Math.abs(curve.rateFor(stick)) <= curve.maxRate(),
                                "exceeded max rate at stick " + stick + " on expo " + expo);
                    }
                }
            }
        }

        /**
         * Bit-for-bit, not within a tolerance. A curve that is only approximately odd means rolling
         * left and rolling right are subtly different aircraft, and the implementation has to be
         * structurally odd rather than accidentally symmetric for this to hold.
         *
         * <p>From {@code i = 1}: at zero the two sides differ only in the sign of a zero, which is
         * not a stick position anyone can hold. {@code centredStickDemandsNoRotation} covers centre.
         */
        @Test
        void isOddSoLeftAndRightFeelIdentical() {
            for (double expo : EXPOS) {
                for (double[] tune : TUNES) {
                    RateCurve curve = RateCurve.fromDegrees(tune[0], tune[1], expo);

                    for (int i = 1; i <= 1000; i++) {
                        double stick = (double) i / 1000.0;

                        assertEquals(-curve.rateFor(stick), curve.rateFor(-stick));
                    }
                }
            }
        }
    }

    @Nested
    class Expo {

        /**
         * The honest form of "expo reduces sensitivity near centre": both endpoints stay pinned and
         * every interior point moves down as expo rises.
         */
        @Test
        void higherExpoLowersTheDemandedRateAtEveryPartialDeflection() {
            for (int i = 1; i < 1000; i++) {
                double stick = (double) i / 1000.0;

                double previous = Double.MAX_VALUE;
                for (double expo : EXPOS) {
                    double rate = DEFAULT.withExpo(expo).rateFor(stick);

                    assertTrue(
                            rate < previous,
                            "expo " + expo + " did not lower the rate at stick " + stick);
                    previous = rate;
                }
            }
        }

        @Test
        void leavesBothEndpointsAloneWhateverTheExpo() {
            for (double expo : EXPOS) {
                RateCurve curve = DEFAULT.withExpo(expo);

                assertEquals(0.0, curve.rateFor(0.0));
                assertEquals(DEFAULT.maxRate(), curve.rateFor(1.0));
            }
        }

        /**
         * The nuance the class javadoc claims and the ticket title obscures: expo shapes the middle
         * of the throw, it does not steepen or soften the very centre. If someone later swaps this
         * family for one where expo <em>does</em> change the centre slope, this fails.
         */
        @Test
        void leavesTheSlopeAtCentreAloneWhateverTheExpo() {
            double h = 1e-8;
            for (double expo : EXPOS) {
                RateCurve curve = DEFAULT.withExpo(expo);

                double slope = (curve.rateFor(h) - curve.rateFor(-h)) / (2.0 * h);

                // The shaped term contributes (M − C)·(1 − e)·h here, so the bound scales with h.
                assertEquals(curve.centreSensitivity(), slope, 1e-6, "expo " + expo);
            }
        }

        /**
         * Zero expo is not a straight line, because an independent centre sensitivity and a pinned
         * endpoint cannot both hold along one. Worth pinning so nobody reads zero expo as "off".
         */
        @Test
        void isStillCurvedAtZeroExpoBecauseTheEndpointIsPinned() {
            RateCurve curve = DEFAULT.withExpo(0.0);

            double halfStick = curve.rateFor(0.5);

            assertNotEquals(0.5 * curve.maxRate(), halfStick);
            assertTrue(halfStick < 0.5 * curve.maxRate(), "expected the curve to sit below linear");
        }
    }

    @Nested
    class LinearReference {

        /**
         * #13's linear stick-to-rate map, recovered. This is the feel-neutral configuration the
         * choice of curve family was argued on, so it is worth being able to point at.
         *
         * <p>A few ULP rather than exact: the grouping that makes full stick exact
         * ({@code C(x − s) + Ms}) cannot also be exact here, where the two terms have to cancel.
         */
        @Test
        void reducesToTheLinearMapWhenCentreSensitivityEqualsMaxRate() {
            for (double expo : EXPOS) {
                RateCurve linear = DEFAULT.withExpo(expo).asLinear();

                for (int i = 0; i <= 2000; i++) {
                    double stick = -1.0 + 2.0 * i / 2000.0;
                    double expected = linear.maxRate() * stick;

                    assertEquals(expected, linear.rateFor(stick), 4.0 * Math.ulp(expected));
                }
            }
        }

        @Test
        void asLinearKeepsTheEndpointAndRaisesCentreSensitivityToIt() {
            RateCurve linear = DEFAULT.asLinear();

            assertEquals(DEFAULT.maxRate(), linear.maxRate());
            assertEquals(DEFAULT.maxRate(), linear.centreSensitivity());
            assertEquals(DEFAULT.expo(), linear.expo());
        }
    }

    @Nested
    class Validation {

        /**
         * Betaflight writes {@code MAX(0, rates − centerSensitivity)}, which silently flattens an
         * inverted tune into a straight line and ignores the max rate the pilot set. Rejecting says
         * so instead.
         */
        @Test
        void rejectsAMaxRateBelowCentreSensitivityBecauseTheCurveWouldNotBeMonotonic() {
            assertThrows(
                    IllegalArgumentException.class, () -> RateCurve.fromDegrees(800, 400, 0.5));
        }

        @Test
        void acceptsAMaxRateEqualToCentreSensitivityBecauseThatIsTheLinearTune() {
            RateCurve curve = RateCurve.fromDegrees(500, 500, 0.5);

            assertEquals(curve.maxRate(), curve.centreSensitivity());
        }

        /**
         * Unlike {@link PidGains}, which permits zero gains because P-only is a real tune. There is
         * no tune in which the sticks do nothing near centre; Betaflight's own minimum is 10 °/s.
         */
        @Test
        void rejectsANonPositiveCentreSensitivityBecauseDeadSticksAreNotATune() {
            assertThrows(IllegalArgumentException.class, () -> RateCurve.fromDegrees(0, 800, 0.5));
            assertThrows(IllegalArgumentException.class, () -> RateCurve.fromDegrees(-10, 800, 0.5));
        }

        @Test
        void rejectsANonPositiveMaxRate() {
            assertThrows(IllegalArgumentException.class, () -> new RateCurve(1.0, 0.0, 0.5));
            assertThrows(IllegalArgumentException.class, () -> new RateCurve(1.0, -1.0, 0.5));
        }

        @Test
        void rejectsExpoOutsideTheUnitRange() {
            assertThrows(IllegalArgumentException.class, () -> RateCurve.fromDegrees(200, 800, -0.01));
            assertThrows(IllegalArgumentException.class, () -> RateCurve.fromDegrees(200, 800, 1.01));
        }

        @Test
        void rejectsNonFiniteParameters() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new RateCurve(Double.NaN, 1.0, 0.5));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new RateCurve(1.0, Double.POSITIVE_INFINITY, 0.5));
            assertThrows(
                    IllegalArgumentException.class, () -> new RateCurve(1.0, 2.0, Double.NaN));
        }

        /**
         * Follows {@link com.maartenpeels.fpv.control.ControlInput}: the only legitimate source of a
         * stick position is already validated, so an out-of-range one is a bug. Clamping instead
         * would quietly break {@code neverDemandsMoreThanTheMaxRate}.
         */
        @Test
        void rejectsAStickPositionBeyondFullDeflection() {
            assertThrows(IllegalArgumentException.class, () -> DEFAULT.rateFor(1.0001));
            assertThrows(IllegalArgumentException.class, () -> DEFAULT.rateFor(-1.0001));
            assertThrows(IllegalArgumentException.class, () -> DEFAULT.rateFor(Double.NaN));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> DEFAULT.rateFor(Double.NEGATIVE_INFINITY));
        }
    }

    @Nested
    class Degrees {

        @Test
        void fromDegreesConvertsBothRatesAndLeavesExpoAlone() {
            RateCurve curve = RateCurve.fromDegrees(200, 800, 0.54);

            assertEquals(Math.toRadians(200), curve.centreSensitivity());
            assertEquals(Math.toRadians(800), curve.maxRate());
            assertEquals(0.54, curve.expo());
        }

        @Test
        void fromDegreesRejectsAnInvertedTuneJustLikeTheConstructor() {
            assertThrows(IllegalArgumentException.class, () -> RateCurve.fromDegrees(800, 200, 0.5));
        }
    }

    @Nested
    class Determinism {

        @Test
        void answersIdenticallyForIdenticalArguments() {
            for (int i = 0; i <= 100; i++) {
                double stick = -1.0 + 2.0 * i / 100.0;

                assertEquals(DEFAULT.rateFor(stick), DEFAULT.rateFor(stick));
            }
        }
    }
}
