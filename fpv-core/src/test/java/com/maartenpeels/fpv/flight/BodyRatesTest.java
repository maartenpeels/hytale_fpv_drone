package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
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
            // Asserted directly rather than componentwise. This used to need three loose assertions
            // because toBodyAxes() negates, and negating 0.0 gave -0.0, which record equality
            // distinguishes from 0.0. #38 fixed that in Vec3's constructor, so Vec3.ZERO is now the
            // honest expectation.
            assertEquals(Vec3.ZERO, BodyRates.ZERO.toBodyAxes());
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

    /**
     * #38. The third record in the family that needs it, and it qualifies on the same two grounds as
     * {@code Vec3} and {@code Quat}: it does its own sign-flipping arithmetic — {@link
     * BodyRates#toBodyAxes()} negates all three components, and {@code QuadIntegrator} applies
     * angular drag as {@code scale(-angularDrag)} — and it has a canonical {@code ZERO} that callers
     * compare against.
     *
     * <p>Bit-level assertions where value-level ones would pass regardless.
     */
    @Nested
    class SignedZero {

        private static final long NEGATIVE_ZERO_BITS = Double.doubleToRawLongBits(-0.0);

        private static void assertNoNegativeZero(BodyRates rates) {
            for (double component : new double[] {rates.roll(), rates.pitch(), rates.yaw()}) {
                assertNotEquals(
                        NEGATIVE_ZERO_BITS,
                        Double.doubleToRawLongBits(component),
                        component + " of " + rates + " must not be negative zero");
            }
        }

        @Test
        void scalingZeroRatesByANegativeStaysEqualToZero() {
            // This is how QuadIntegrator applies angular drag, and a drone hovering level is at
            // exactly zero rates -- so it is the common case, not an edge case.
            assertEquals(BodyRates.ZERO, BodyRates.ZERO.scale(-1));
            assertEquals(BodyRates.ZERO.hashCode(), BodyRates.ZERO.scale(-1).hashCode());
        }

        @Test
        void scalingByZeroStaysEqualToZeroWhateverTheSignOfTheInput() {
            assertEquals(BodyRates.ZERO, new BodyRates(-1, -2, -3).scale(0));
            assertEquals(BodyRates.ZERO, new BodyRates(1, 2, 3).scale(0));
        }

        @Test
        void everyOperationThatCanLandOnAZeroKeepsItPositive() {
            assertNoNegativeZero(BodyRates.ZERO.scale(-1));
            assertNoNegativeZero(new BodyRates(-1, -2, -3).scale(0));
            assertNoNegativeZero(new BodyRates(-0.0, -0.0, -0.0));
            assertNoNegativeZero(new BodyRates(1, 2, 3).plus(new BodyRates(-1, -2, -3)));
        }

        @Test
        void normalisationDoesNotDisturbNonZeroRates() {
            // The fix must only ever change the sign of a zero -- an altered rate would be altered
            // flight behaviour.
            BodyRates rates = new BodyRates(-0.1, 1.0 / 3.0, -12.5);

            assertEquals(Double.doubleToRawLongBits(-0.1), Double.doubleToRawLongBits(rates.roll()));
            assertEquals(
                    Double.doubleToRawLongBits(1.0 / 3.0), Double.doubleToRawLongBits(rates.pitch()));
            assertEquals(Double.doubleToRawLongBits(-12.5), Double.doubleToRawLongBits(rates.yaw()));
        }
    }
}
