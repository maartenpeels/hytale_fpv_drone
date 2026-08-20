package com.maartenpeels.fpv.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
}
