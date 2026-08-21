package com.maartenpeels.fpv.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class QuatTest {

    private static final double TOLERANCE = 1e-12;

    private static void assertVectorEquals(Vec3 expected, Vec3 actual, double tolerance) {
        assertEquals(expected.x(), actual.x(), tolerance, "x");
        assertEquals(expected.y(), actual.y(), tolerance, "y");
        assertEquals(expected.z(), actual.z(), tolerance, "z");
    }

    @Nested
    class Rotation {

        @Test
        void identityLeavesAVectorAlone() {
            assertVectorEquals(new Vec3(1, 2, 3), Quat.IDENTITY.rotate(new Vec3(1, 2, 3)), TOLERANCE);
        }

        @Test
        void aQuarterTurnAboutUpTakesRightToBackwards() {
            // Right-hand rule about +Y: +X sweeps toward -Z, which is the model's forward. Getting
            // this backwards would make every yaw input turn the drone the wrong way.
            Quat quarterTurn = Quat.fromAxisAngle(Vec3.UP, Math.PI / 2);

            assertVectorEquals(new Vec3(0, 0, -1), quarterTurn.rotate(new Vec3(1, 0, 0)), 1e-15);
        }

        @Test
        void rotationPreservesLength() {
            Quat rotation = Quat.fromAxisAngle(new Vec3(1, 2, 3), 1.234);

            assertEquals(
                    new Vec3(4, -5, 6).length(),
                    rotation.rotate(new Vec3(4, -5, 6)).length(),
                    TOLERANCE);
        }

        @Test
        void inverseRotateUndoesRotate() {
            Quat rotation = Quat.fromAxisAngle(new Vec3(-1, 0.5, 2), -2.1);
            Vec3 original = new Vec3(7, -3, 0.5);

            assertVectorEquals(original, rotation.inverseRotate(rotation.rotate(original)), 1e-14);
        }

        @Test
        void timesAppliesTheRightHandOperandFirst() {
            Quat aboutY = Quat.fromAxisAngle(Vec3.UP, Math.PI / 2);
            Quat aboutX = Quat.fromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2);

            // Applying aboutY first sends +X to -Z; aboutX then sends -Z to +Y.
            assertVectorEquals(
                    new Vec3(0, 1, 0), aboutX.times(aboutY).rotate(new Vec3(1, 0, 0)), 1e-15);
        }
    }

    @Nested
    class Normalisation {

        @Test
        void normalisedHasUnitNorm() {
            assertEquals(1.0, new Quat(1, 2, 3, 4).normalised().norm(), TOLERANCE);
        }

        @Test
        void normalisingACollapsedQuaternionFallsBackToIdentity() {
            // A zero quaternion has no direction to preserve, and returning NaN would corrupt the
            // drone's attitude for the rest of the flight.
            assertEquals(Quat.IDENTITY, new Quat(0, 0, 0, 0).normalised());
            assertEquals(Quat.IDENTITY, new Quat(Double.NaN, 0, 0, 0).normalised());
        }

        @Test
        void fromAxisAngleWithAZeroAxisIsTheIdentity() {
            assertEquals(Quat.IDENTITY, Quat.fromAxisAngle(Vec3.ZERO, 1.0));
        }
    }

    @Nested
    class Integration {

        @Test
        void integratingAConstantRateMatchesTheEquivalentAxisAngleRotation() {
            Vec3 axis = new Vec3(0, 1, 0);
            double rate = 1.5;
            double totalTime = 0.4;
            int steps = 20_000;

            Quat integrated = Quat.IDENTITY;
            for (int i = 0; i < steps; i++) {
                integrated = integrated.integrate(axis.scale(rate), totalTime / steps);
            }

            Vec3 expected = Quat.fromAxisAngle(axis, rate * totalTime).rotate(new Vec3(1, 0, 0));
            assertVectorEquals(expected, integrated.rotate(new Vec3(1, 0, 0)), 1e-9);
        }

        @Test
        void staysUnitLengthOverManyStepsSoRotationRateDoesNotDrift() {
            // The first-order update leaves the quaternion just off the unit sphere every step.
            // Left to accumulate, that scales the drone's apparent rotation rate.
            Quat orientation = Quat.IDENTITY;
            for (int i = 0; i < 100_000; i++) {
                orientation = orientation.integrate(new Vec3(1, 2, -3), 1.0 / 240);
            }

            assertEquals(1.0, orientation.norm(), 1e-12);
        }

        @Test
        void aZeroRateLeavesOrientationUnchanged() {
            Quat rotation = Quat.fromAxisAngle(new Vec3(1, 1, 1), 0.7);
            Vec3 probe = new Vec3(1, -2, 3);

            assertVectorEquals(
                    rotation.rotate(probe),
                    rotation.integrate(Vec3.ZERO, 1.0 / 240).rotate(probe),
                    TOLERANCE);
        }
    }

    /**
     * #38. Same hazard as {@code Vec3}: a record compares {@code double}s with
     * {@code Double.compare}, so a {@code −0.0} component makes an arithmetically correct rotation
     * unequal to the literal it should match. {@link Quat#conjugate()} is the operation that hits it,
     * and {@code IDENTITY} is the value callers compare against.
     *
     * <p>Bit-level assertions where the value-level ones would pass regardless: {@code assertEquals}
     * on {@code double} uses {@code ==}, for which {@code −0.0} and {@code 0.0} are equal.
     */
    @Nested
    class SignedZero {

        private static final long NEGATIVE_ZERO_BITS = Double.doubleToRawLongBits(-0.0);

        private static void assertNoNegativeZero(Quat quaternion) {
            double[] components = {
                quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()
            };
            for (double component : components) {
                assertNotEquals(
                        NEGATIVE_ZERO_BITS,
                        Double.doubleToRawLongBits(component),
                        component + " of " + quaternion + " must not be negative zero");
            }
        }

        @Test
        void conjugatingTheIdentityGivesBackSomethingEqualToTheIdentity() {
            // Without the fix this is (1, −0.0, −0.0, −0.0): numerically the identity rotation, but
            // unequal to Quat.IDENTITY and hashing differently.
            assertEquals(Quat.IDENTITY, Quat.IDENTITY.conjugate());
            assertEquals(Quat.IDENTITY.hashCode(), Quat.IDENTITY.conjugate().hashCode());
        }

        @Test
        void conjugatingTwiceReturnsTheOriginalForRotationsWithZeroComponents() {
            // A yaw-only rotation has two exactly-zero axis components, so it is the case most like
            // the axis-aligned normals that motivated #38.
            Quat yaw = Quat.fromAxisAngle(new Vec3(0, 1, 0), 0.9);

            assertEquals(yaw, yaw.conjugate().conjugate());
            assertNoNegativeZero(yaw.conjugate());
        }

        @Test
        void rotatingAnAxisVectorYieldsAVectorFreeOfNegativeZero() {
            // Quat.rotate() builds its result through Vec3, so this is really a check that the two
            // fixes compose -- SweptResult's javadoc invites callers to rotate a normal and rebuild.
            Quat yaw = Quat.fromAxisAngle(new Vec3(0, 1, 0), Math.PI / 2);

            assertEquals(new Vec3(0, 1, 0), yaw.rotate(new Vec3(0, 1, 0)));
            assertEquals(new Vec3(0, -1, 0), yaw.rotate(new Vec3(0, -1, 0)));
        }

        @Test
        void everyOperationThatCanLandOnAZeroKeepsItPositive() {
            assertNoNegativeZero(Quat.IDENTITY.conjugate());
            assertNoNegativeZero(new Quat(1, -0.0, -0.0, -0.0));
            assertNoNegativeZero(Quat.fromAxisAngle(new Vec3(1, 0, 0), 0.4).conjugate());
            assertNoNegativeZero(Quat.IDENTITY.integrate(Vec3.ZERO, 1.0 / 240));
            assertNoNegativeZero(Quat.fromAxisAngle(Vec3.ZERO, 1.0));
        }

        @Test
        void normalisationDoesNotDisturbNonZeroComponentsSoNoRotationChanges() {
            // The safety argument for a constructor-level fix: it can only change the sign of a zero.
            Quat quaternion = new Quat(0.5, -0.5, 1.0 / 3.0, -0.1);

            assertEquals(Double.doubleToRawLongBits(0.5), Double.doubleToRawLongBits(quaternion.w()));
            assertEquals(
                    Double.doubleToRawLongBits(-0.5), Double.doubleToRawLongBits(quaternion.x()));
            assertEquals(
                    Double.doubleToRawLongBits(1.0 / 3.0),
                    Double.doubleToRawLongBits(quaternion.y()));
            assertEquals(
                    Double.doubleToRawLongBits(-0.1), Double.doubleToRawLongBits(quaternion.z()));
        }

        @Test
        void aConjugateStillInvertsItsRotationSoTheFixChangedNoArithmetic() {
            // The property that would break if normalisation touched anything but a zero's sign.
            Quat rotation = Quat.fromAxisAngle(new Vec3(0, 1, 0), 1.1);
            Vec3 probe = new Vec3(1, 0, -2);

            assertVectorEquals(probe, rotation.inverseRotate(rotation.rotate(probe)), TOLERANCE);
        }
    }
}
