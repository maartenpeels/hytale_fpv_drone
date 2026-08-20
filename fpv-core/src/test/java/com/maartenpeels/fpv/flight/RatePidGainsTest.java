package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class RatePidGainsTest {

    @Nested
    class DefaultTune {

        @Test
        void reproducesThirteensProportionalTrackerExactly() {
            // The default P gains are not a guess: they are the placeholder rate tracker #13 shipped,
            // rewritten as a gain. That is what makes adding the rate loop feel-neutral -- the drone
            // pulls toward a demanded rate exactly as hard as it did before, and merely gains the
            // ability to arrive.
            double expectedRollPitch =
                    1.0
                            / (RatePidGains.DEFAULT_RATE_TIME_CONSTANT
                                    * QuadParameters.DEFAULT.rollPitchAuthority());

            assertEquals(expectedRollPitch, RatePidGains.DEFAULT.roll().proportional(), 1e-12);
            assertEquals(expectedRollPitch, RatePidGains.DEFAULT.pitch().proportional(), 1e-12);
        }

        @Test
        void givesYawAHigherGainBecauseItsAuthorityIsWeaker() {
            // Not a preference -- an inevitability. Yaw authority is a quarter of roll and pitch, so
            // the same rate error has to buy four times as much of the yaw axis' available torque.
            // A single shared gain could not do this, which is why the tune is per axis.
            assertTrue(
                    RatePidGains.DEFAULT.yaw().proportional()
                            > RatePidGains.DEFAULT.roll().proportional() * 3.5,
                    "yaw gain "
                            + RatePidGains.DEFAULT.yaw().proportional()
                            + " should be several times roll's "
                            + RatePidGains.DEFAULT.roll().proportional());
            assertEquals(
                    QuadParameters.DEFAULT.rollPitchAuthority()
                            / QuadParameters.DEFAULT.yawAuthority(),
                    RatePidGains.DEFAULT.yaw().proportional()
                            / RatePidGains.DEFAULT.roll().proportional(),
                    1e-9);
        }

        @Test
        void shipsWithNoDerivativeActionBecauseThePlantHasNoLagToAnticipate() {
            // Measured, not assumed: every derivative time tried made overshoot, stick-release
            // bounce and disturbance recovery worse on this airframe. If this assertion ever needs
            // changing, the model has grown some lag -- which is the interesting part, not the test.
            assertEquals(0.0, RatePidGains.DEFAULT.roll().derivative(), 1e-12);
            assertEquals(0.0, RatePidGains.DEFAULT.pitch().derivative(), 1e-12);
            assertEquals(0.0, RatePidGains.DEFAULT.yaw().derivative(), 1e-12);
        }

        @Test
        void shipsWithAnIntegralTermOnEveryAxis() {
            assertTrue(RatePidGains.DEFAULT.roll().integral() > 0);
            assertTrue(RatePidGains.DEFAULT.pitch().integral() > 0);
            assertTrue(RatePidGains.DEFAULT.yaw().integral() > 0);
        }

        @Test
        void boundsTheIntegralOnEveryAxis() {
            // An unbounded axis is the windup bug, so it is worth an assertion rather than trust.
            assertTrue(RatePidGains.DEFAULT.roll().integralLimit() > 0);
            assertTrue(RatePidGains.DEFAULT.roll().integralLimit() < 1);
            assertTrue(RatePidGains.DEFAULT.yaw().integralLimit() < 1);
        }
    }

    @Nested
    class Variants {

        @Test
        void uniformPutsTheSameGainsOnAllThreeAxes() {
            PidGains gains = new PidGains(0.2, 1.5, 0.003, 0.3);
            RatePidGains uniform = RatePidGains.uniform(gains);

            assertEquals(gains, uniform.roll());
            assertEquals(gains, uniform.pitch());
            assertEquals(gains, uniform.yaw());
        }

        @Test
        void onEveryAxisAppliesTheChangeWithoutFlatteningTheDifferencesBetweenAxes() {
            RatePidGains doubled =
                    RatePidGains.DEFAULT.onEveryAxis(g -> g.withProportional(g.proportional() * 2));

            assertEquals(
                    RatePidGains.DEFAULT.roll().proportional() * 2,
                    doubled.roll().proportional(),
                    1e-12);
            assertEquals(
                    RatePidGains.DEFAULT.yaw().proportional() * 2,
                    doubled.yaw().proportional(),
                    1e-12);
            assertTrue(
                    doubled.yaw().proportional() > doubled.roll().proportional(),
                    "yaw should still be the higher-gain axis afterwards");
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsAMissingAxis() {
            PidGains gains = new PidGains(0.2, 1.5, 0.003, 0.3);

            assertThrows(
                    IllegalArgumentException.class, () -> new RatePidGains(null, gains, gains));
            assertThrows(
                    IllegalArgumentException.class, () -> new RatePidGains(gains, null, gains));
            assertThrows(
                    IllegalArgumentException.class, () -> new RatePidGains(gains, gains, null));
        }
    }
}
