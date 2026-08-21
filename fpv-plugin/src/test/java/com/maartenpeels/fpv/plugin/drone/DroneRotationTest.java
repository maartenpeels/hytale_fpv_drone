package com.maartenpeels.fpv.plugin.drone;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.hypixel.hytale.math.vector.Rotation3f;
import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The quaternion-to-Euler boundary, where a sign error is invisible in code and obvious in flight.
 *
 * <p>No harness: {@code Rotation3f} and JOML are plain math. Every sign here is pinned against the
 * <em>axis convention</em> — which way the nose actually ends up pointing — rather than against
 * another expression in the same file, because two expressions with the same transposition agree with
 * each other perfectly.
 */
class DroneRotationTest {

    private static final double QUARTER_TURN = Math.PI / 2.0;
    private static final double EIGHTH_TURN = Math.PI / 4.0;

    /** Core's forward axis. {@code DroneState} documents forward as {@code −Z}. */
    private static final Vec3 FORWARD = new Vec3(0, 0, -1);

    /** Hytale's own direction vector for a yaw/pitch, from {@code Vector3dUtil.setYawPitch}. */
    private static Vec3 hytaleDirection(double yaw, double pitch) {
        return new Vec3(
                -Math.sin(yaw) * Math.cos(pitch), Math.sin(pitch), -Math.cos(yaw) * Math.cos(pitch));
    }

    /**
     * Hytale's direction for the angles a {@code Rotation3f} actually <em>stored</em>.
     *
     * <p>{@code Rotation3f}'s fields are {@code float}, so the expected direction has to be built from
     * the narrowed values rather than from the doubles that went in — otherwise every assertion here
     * carries a ~1e-8 float-narrowing error and its tolerance has to be loose enough to hide a real
     * sign or axis mistake in the small-angle cases.
     */
    private static Vec3 storedDirection(Rotation3f rotation) {
        return hytaleDirection(rotation.yaw(), rotation.pitch());
    }

    /**
     * Tolerance note: {@code 1e-6}, not tighter.
     *
     * <p>{@code Rotation3f} stores {@code float}, so an angle carries ~1e-7 of absolute error, and near
     * a zero crossing (yaw = π, where {@code sin} is 0 and its derivative is 1) that lands directly on
     * the direction component. The two routes to it — JOML's quaternion composition and the closed-form
     * {@code setYawPitch} — then cancel differently, at the 1e-9 level here. None of that can hide what
     * this file is looking for: a sign flip, an axis transposition or a degree/radian slip all move a
     * component by an order-1 amount.
     */
    private static void assertVecEquals(Vec3 expected, Vec3 actual) {
        assertEquals(expected.x(), actual.x(), 1e-6, "x of " + actual);
        assertEquals(expected.y(), actual.y(), 1e-6, "y of " + actual);
        assertEquals(expected.z(), actual.z(), 1e-6, "z of " + actual);
    }

    @Nested
    class AgreesWithHytalesOwnDirectionVector {

        @Test
        void forYawBecauseHytaleYawIsARightHandedRotationAboutPlusY() {
            // The claim CLAUDE.md gets wrong ("opposite to a right-handed yaw about +Y"). If it were
            // right, this test would fail on every non-zero yaw -- so this is the assertion that
            // decides whether the conversion needs a negation, and it says it does not.
            for (double yaw : new double[] {0.0, EIGHTH_TURN, QUARTER_TURN, 2.0, -1.3, Math.PI}) {
                Rotation3f rotation = new Rotation3f(0f, (float) yaw, 0f);
                Quat attitude = DroneRotation.toQuat(rotation);
                assertVecEquals(storedDirection(rotation), attitude.rotate(FORWARD));
            }
        }

        @Test
        void forPitchBecauseHytalePitchIsPositiveNoseUpAndSoIsARightHandedRotationAboutPlusX() {
            for (double pitch : new double[] {0.0, EIGHTH_TURN, -EIGHTH_TURN, 1.2, -1.2}) {
                Rotation3f rotation = new Rotation3f((float) pitch, 0f, 0f);
                Quat attitude = DroneRotation.toQuat(rotation);
                assertVecEquals(storedDirection(rotation), attitude.rotate(FORWARD));
            }
        }

        @Test
        void forYawAndPitchTogether() {
            Rotation3f rotation = new Rotation3f(0.4f, 1.1f, 0f);
            Quat attitude = DroneRotation.toQuat(rotation);
            assertVecEquals(storedDirection(rotation), attitude.rotate(FORWARD));
        }

        @Test
        void positivePitchRaisesTheNoseWhichIsTheOppositeOfControlInputsConvention() {
            // Spelled out because CLAUDE.md says a rotation adapter "must negate" pitch. That rule is
            // about ControlInput (transmitter convention, positive nose-down); between Quat and
            // Rotation3f both sides are geometric and nothing is negated. Getting this backwards would
            // put the drone's rendered attitude upside down relative to its simulated one.
            Quat noseUp = DroneRotation.toQuat(new Rotation3f((float) EIGHTH_TURN, 0f, 0f));
            assertTrue(noseUp.rotate(FORWARD).y() > 0.0, "positive Hytale pitch must point upward");
        }
    }

    @Nested
    class RoundTrips {

        private static void assertRoundTrips(float pitch, float yaw, float roll) {
            Rotation3f original = new Rotation3f(pitch, yaw, roll);
            Rotation3f back = DroneRotation.toRotation(DroneRotation.toQuat(original));

            assertEquals(original.pitch(), back.pitch(), 1e-5, "pitch");
            assertEquals(original.yaw(), back.yaw(), 1e-5, "yaw");
            assertEquals(original.roll(), back.roll(), 1e-5, "roll");
        }

        @Test
        void yawAlone() {
            assertRoundTrips(0f, 1.1f, 0f);
        }

        @Test
        void pitchAlone() {
            assertRoundTrips(0.4f, 0f, 0f);
        }

        @Test
        void rollAlone() {
            // Roll is the axis the whole feature exists for -- decision 4 attaches the camera with
            // full roll -- and it is the one Hytale's own character controller never produces.
            assertRoundTrips(0f, 0f, 0.9f);
        }

        @Test
        void allThreeTogetherWithoutTransposingThem() {
            // Three distinct magnitudes on purpose: with equal angles a pitch/yaw/roll transposition
            // round-trips perfectly and proves nothing.
            assertRoundTrips(0.2f, 0.7f, -1.1f);
        }
    }

    @Nested
    class WriteTo {

        @Test
        void writesTheSameAnglesAsTheAllocatingFormSoTheHotPathIsNotADifferentConversion() {
            Quat attitude = DroneRotation.toQuat(new Rotation3f(0.2f, 0.7f, -1.1f));

            Rotation3f allocated = DroneRotation.toRotation(attitude);
            Rotation3f reused = new Rotation3f(9f, 9f, 9f);
            DroneRotation.writeTo(attitude, reused);

            assertEquals(allocated.pitch(), reused.pitch(), 0f);
            assertEquals(allocated.yaw(), reused.yaw(), 0f);
            assertEquals(allocated.roll(), reused.roll(), 0f);
        }

        @Test
        void overwritesEveryAxisSoAReusedScratchCannotLeakTheLastDronesAttitude() {
            // AdvanceFlight reuses one Rotation3f for every drone in the store. If writeTo left an
            // axis untouched, drone N would inherit drone N-1's roll.
            Rotation3f dirty = new Rotation3f(9f, 9f, 9f);

            DroneRotation.writeTo(Quat.IDENTITY, dirty);

            assertEquals(0f, dirty.pitch(), 1e-6);
            assertEquals(0f, dirty.yaw(), 1e-6);
            assertEquals(0f, dirty.roll(), 1e-6);
        }
    }

    @Nested
    class HeadingOf {

        @Test
        void keepsTheYawSoADroneArmsFacingWhereItsPilotFaced() {
            Rotation3f pilotLook = new Rotation3f(0.4f, 1.1f, -0.7f);
            Quat heading = DroneRotation.headingOf(pilotLook);
            assertVecEquals(hytaleDirection(pilotLook.yaw(), 0.0), heading.rotate(FORWARD));
        }

        @Test
        void dropsPitchAndRollSoADroneNeverArmsAlreadyBanked() {
            Rotation3f back = DroneRotation.toRotation(
                    DroneRotation.headingOf(new Rotation3f(0.4f, 1.1f, -0.7f)));

            assertEquals(0f, back.pitch(), 1e-5);
            assertEquals(0f, back.roll(), 1e-5);
            assertEquals(1.1f, back.yaw(), 1e-5);
        }
    }
}
