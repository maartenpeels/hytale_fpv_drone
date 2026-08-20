package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.ControlInput;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class RateProfileTest {

    @Nested
    class Demand {

        @Test
        void centredSticksDemandNoRotationOnAnyAxis() {
            BodyRates demanded = RateProfile.DEFAULT.demand(ControlInput.NEUTRAL);

            assertEquals(0.0, demanded.roll());
            assertEquals(0.0, demanded.pitch());
            assertEquals(0.0, demanded.yaw());
        }

        @Test
        void fullSticksDemandExactlyTheProfilesMaxRates() {
            BodyRates demanded = RateProfile.DEFAULT.demand(new ControlInput(0f, 1f, 1f, 1f));

            assertEquals(RateProfile.DEFAULT_ROLL_PITCH_MAX_RATE, demanded.roll());
            assertEquals(RateProfile.DEFAULT_ROLL_PITCH_MAX_RATE, demanded.pitch());
            assertEquals(RateProfile.DEFAULT_YAW_MAX_RATE, demanded.yaw());
        }

        @Test
        void fullNegativeSticksDemandTheSameRatesInverted() {
            BodyRates demanded = RateProfile.DEFAULT.demand(new ControlInput(0f, -1f, -1f, -1f));

            assertEquals(-RateProfile.DEFAULT_ROLL_PITCH_MAX_RATE, demanded.roll());
            assertEquals(-RateProfile.DEFAULT_ROLL_PITCH_MAX_RATE, demanded.pitch());
            assertEquals(-RateProfile.DEFAULT_YAW_MAX_RATE, demanded.yaw());
        }

        /**
         * Each stick axis drives its own axis and no other. Too obvious to state, and invisible in
         * flight until a pilot wonders why pitch rolls the aircraft — so it is checked with three
         * curves that cannot be confused for one another.
         */
        @Test
        void mapsEachStickAxisOntoItsOwnAxisAndNoOther() {
            RateProfile profile =
                    new RateProfile(
                            RateCurve.fromDegrees(100, 100, 0.0),
                            RateCurve.fromDegrees(200, 200, 0.0),
                            RateCurve.fromDegrees(400, 400, 0.0));

            BodyRates rollOnly = profile.demand(new ControlInput(0f, 1f, 0f, 0f));
            assertEquals(Math.toRadians(100), rollOnly.roll());
            assertEquals(0.0, rollOnly.pitch());
            assertEquals(0.0, rollOnly.yaw());

            BodyRates pitchOnly = profile.demand(new ControlInput(0f, 0f, 1f, 0f));
            assertEquals(0.0, pitchOnly.roll());
            assertEquals(Math.toRadians(200), pitchOnly.pitch());
            assertEquals(0.0, pitchOnly.yaw());

            BodyRates yawOnly = profile.demand(new ControlInput(0f, 0f, 0f, 1f));
            assertEquals(0.0, yawOnly.roll());
            assertEquals(0.0, yawOnly.pitch());
            assertEquals(Math.toRadians(400), yawOnly.yaw());
        }

        @Test
        void ignoresThrottleBecauseThrottleIsNotAnAngularRate() {
            ControlInput closed = new ControlInput(0f, 0.5f, -0.5f, 0.25f);
            ControlInput open = new ControlInput(1f, 0.5f, -0.5f, 0.25f);

            assertEquals(RateProfile.DEFAULT.demand(closed), RateProfile.DEFAULT.demand(open));
        }

        @Test
        void rejectsANullInput() {
            assertThrows(
                    IllegalArgumentException.class, () -> RateProfile.DEFAULT.demand(null));
        }

        @Test
        void alwaysProducesFiniteRatesSoTheRatePidWillAcceptThem() {
            for (int i = 0; i <= 200; i++) {
                float stick = (float) (-1.0 + 2.0 * i / 200.0);
                BodyRates demanded =
                        RateProfile.DEFAULT.demand(new ControlInput(0f, stick, stick, stick));

                assertTrue(demanded.isFinite(), "not finite at stick " + stick);
            }
        }
    }

    @Nested
    class Defaults {

        @Test
        void yawIsSofterThanRollAndPitchBecauseAQuadYawsOnPropReaction() {
            RateProfile profile = RateProfile.DEFAULT;

            assertTrue(profile.yaw().maxRate() < profile.roll().maxRate());
            assertTrue(profile.yaw().centreSensitivity() < profile.roll().centreSensitivity());
        }

        @Test
        void rollAndPitchAreTheSameCurveBecauseTheAxesAreSymmetric() {
            assertEquals(RateProfile.DEFAULT.roll(), RateProfile.DEFAULT.pitch());
        }

        /**
         * #13 chose these endpoints deliberately and #24 judges them, so #15 inherits rather than
         * re-picks them. If this fails, someone changed the tune's feel while meaning to change its
         * shape.
         */
        @Test
        void inheritsThirteensEndpointsUnchanged() {
            assertEquals(800.0, Math.toDegrees(RateProfile.DEFAULT.roll().maxRate()), 1e-12);
            assertEquals(800.0, Math.toDegrees(RateProfile.DEFAULT.pitch().maxRate()), 1e-12);
            assertEquals(400.0, Math.toDegrees(RateProfile.DEFAULT.yaw().maxRate()), 1e-12);
        }

        @Test
        void holdsTheSameCentreToMaxRatioOnEveryAxis() {
            RateProfile profile = RateProfile.DEFAULT;

            double rollRatio = profile.roll().centreSensitivity() / profile.roll().maxRate();
            double yawRatio = profile.yaw().centreSensitivity() / profile.yaw().maxRate();

            assertEquals(rollRatio, yawRatio, 1e-12);
        }

        @Test
        void maxRatesReportsTheFullStickRateOfEveryAxis() {
            BodyRates maxRates = RateProfile.DEFAULT.maxRates();

            assertEquals(RateProfile.DEFAULT.roll().maxRate(), maxRates.roll());
            assertEquals(RateProfile.DEFAULT.pitch().maxRate(), maxRates.pitch());
            assertEquals(RateProfile.DEFAULT.yaw().maxRate(), maxRates.yaw());
        }

        @Test
        void maxRatesAgreesWithWhatFullStickActuallyDemands() {
            BodyRates atFullStick = RateProfile.DEFAULT.demand(new ControlInput(0f, 1f, 1f, 1f));

            assertEquals(RateProfile.DEFAULT.maxRates(), atFullStick);
        }
    }

    @Nested
    class Construction {

        @Test
        void rejectsAMissingCurveOnAnyAxis() {
            RateCurve curve = RateProfile.DEFAULT.roll();

            assertThrows(
                    IllegalArgumentException.class, () -> new RateProfile(null, curve, curve));
            assertThrows(
                    IllegalArgumentException.class, () -> new RateProfile(curve, null, curve));
            assertThrows(
                    IllegalArgumentException.class, () -> new RateProfile(curve, curve, null));
        }

        @Test
        void uniformPutsTheSameCurveOnAllThreeAxes() {
            RateCurve curve = RateCurve.fromDegrees(150, 600, 0.3);

            RateProfile profile = RateProfile.uniform(curve);

            assertEquals(curve, profile.roll());
            assertEquals(curve, profile.pitch());
            assertEquals(curve, profile.yaw());
        }

        @Test
        void onEveryAxisAppliesTheChangeToAllThreeCurves() {
            RateProfile flattened = RateProfile.DEFAULT.onEveryAxis(curve -> curve.withExpo(0.0));

            assertEquals(0.0, flattened.roll().expo());
            assertEquals(0.0, flattened.pitch().expo());
            assertEquals(0.0, flattened.yaw().expo());
            assertEquals(RateProfile.DEFAULT.maxRates(), flattened.maxRates());
        }
    }
}
