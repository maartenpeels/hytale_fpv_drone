package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class BodyRatesTest {

    /** A tenth of a second of rotation — enough that a sign is unmistakable. */
    private static final double SPIN_SECONDS = 0.1;

    private static Quat afterSpinning(BodyRates rates) {
        return Quat.IDENTITY.integrate(rates.toBodyAxes(), SPIN_SECONDS);
    }

    @Nested
    class SignConvention {

        // These four are the entire point of the type. They are asserted as physical statements
        // about where an axis ends up, not as expected component values, because a component value
        // can be "right" against a mistaken derivation while the drone still rolls the wrong way.

        @Test
        void positiveRollBanksRightByDroppingTheRightHandSide() {
            Vec3 right = afterSpinning(new BodyRates(1, 0, 0)).rotate(new Vec3(1, 0, 0));

            assertTrue(right.y() < 0, "right-hand side should drop, but y was " + right.y());
        }

        @Test
        void positivePitchPutsTheNoseDownFollowingTransmitterConvention() {
            // Not aerospace convention. See ControlInput's note -- do not "correct" this.
            Vec3 nose = afterSpinning(new BodyRates(0, 1, 0)).rotate(new Vec3(0, 0, -1));

            assertTrue(nose.y() < 0, "nose should drop, but y was " + nose.y());
        }

        @Test
        void positiveYawTurnsTheNoseTowardsTheBodysRightSide() {
            Vec3 nose = afterSpinning(new BodyRates(0, 0, 1)).rotate(new Vec3(0, 0, -1));

            assertTrue(nose.x() > 0, "nose should swing to +X, but x was " + nose.x());
        }

        @Test
        void eachAxisRotatesOnlyItsOwnAxis() {
            // Cross-coupling here would be indistinguishable from a badly tuned airframe in flight.
            Vec3 rollAxisUnderRoll = afterSpinning(new BodyRates(1, 0, 0)).rotate(new Vec3(0, 0, -1));
            Vec3 pitchAxisUnderPitch = afterSpinning(new BodyRates(0, 1, 0)).rotate(new Vec3(1, 0, 0));
            Vec3 yawAxisUnderYaw = afterSpinning(new BodyRates(0, 0, 1)).rotate(new Vec3(0, 1, 0));

            assertEquals(-1.0, rollAxisUnderRoll.z(), 1e-12, "roll must leave the nose pointing on");
            assertEquals(1.0, pitchAxisUnderPitch.x(), 1e-12, "pitch must leave the right axis on");
            assertEquals(1.0, yawAxisUnderYaw.y(), 1e-12, "yaw must leave the up axis on");
        }

        @Test
        void mapsOntoNegativeBodyAxesBecausePilotSignsOpposeTheRightHandRule() {
            assertEquals(new Vec3(-2, -3, -1), new BodyRates(1, 2, 3).toBodyAxes());
        }

        @Test
        void zeroRatesMapToAZeroAxisVector() {
            // Componentwise, because negating 0.0 yields -0.0 and record equality distinguishes
            // the two. Harmless downstream -- Quat.integrate multiplies it away -- but not worth
            // asserting exactly.
            Vec3 axes = BodyRates.ZERO.toBodyAxes();

            assertEquals(0.0, axes.x(), 0.0);
            assertEquals(0.0, axes.y(), 0.0);
            assertEquals(0.0, axes.z(), 0.0);
        }
    }

    @Nested
    class Algebra {

        @Test
        void addsAndScalesPerAxis() {
            assertEquals(
                    new BodyRates(4, 6, 8), new BodyRates(1, 2, 3).plus(new BodyRates(3, 4, 5)));
            assertEquals(new BodyRates(2, 4, 6), new BodyRates(1, 2, 3).scale(2));
        }

        @Test
        void detectsNonFiniteRates() {
            assertTrue(new BodyRates(1, 2, 3).isFinite());
            assertFalse(new BodyRates(Double.NaN, 0, 0).isFinite());
            assertFalse(new BodyRates(0, Double.POSITIVE_INFINITY, 0).isFinite());
        }
    }
}
