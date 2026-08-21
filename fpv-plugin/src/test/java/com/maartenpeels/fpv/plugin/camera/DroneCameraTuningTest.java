package com.maartenpeels.fpv.plugin.camera;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DroneCameraTuningTest {

    private static final float EPSILON = 1.0e-6f;

    private static DroneCameraTuning tuning(float posLerp, float rotLerp, float uptilt) {
        return new DroneCameraTuning(DroneCameraView.TRACKED, posLerp, rotLerp, true, 0.1, uptilt);
    }

    @Nested
    class Validation {

        @Test
        void rejectsNonFiniteLerpSpeedsBecauseNaNRendersAsACameraThatNeverMoves() {
            // The dangerous case. A NaN here does not crash -- it produces a camera that appears to
            // ignore the setting, which sends someone hunting a protocol bug that is not there.
            assertThrows(IllegalArgumentException.class, () -> tuning(Float.NaN, 0.2f, 0.0f));
            assertThrows(IllegalArgumentException.class, () -> tuning(0.2f, Float.NaN, 0.0f));
            assertThrows(IllegalArgumentException.class,
                    () -> tuning(Float.POSITIVE_INFINITY, 0.2f, 0.0f));
        }

        @Test
        void rejectsNonFiniteUptiltBecauseNaNRendersAsNoTiltAtAll() {
            // Same failure shape as the degrees-for-radians mistake that nearly derailed #28: it
            // reads as "roll is ignored" rather than as a bad input.
            assertThrows(IllegalArgumentException.class, () -> tuning(0.2f, 0.2f, Float.NaN));
            assertThrows(IllegalArgumentException.class,
                    () -> tuning(0.2f, 0.2f, Float.NEGATIVE_INFINITY));
        }

        @Test
        void rejectsNonPositiveLerpSpeeds() {
            assertThrows(IllegalArgumentException.class, () -> tuning(0.0f, 0.2f, 0.0f));
            assertThrows(IllegalArgumentException.class, () -> tuning(0.2f, 0.0f, 0.0f));
            assertThrows(IllegalArgumentException.class, () -> tuning(-0.2f, 0.2f, 0.0f));
        }

        @Test
        void rejectsNonFiniteEyeHeight() {
            assertThrows(IllegalArgumentException.class, () -> new DroneCameraTuning(
                    DroneCameraView.TRACKED, 0.2f, 0.2f, true, Double.NaN, 0.0f));
        }

        @Test
        void rejectsANullView() {
            assertThrows(IllegalArgumentException.class,
                    () -> new DroneCameraTuning(null, 0.2f, 0.2f, true, 0.1, 0.0f));
        }

        @Test
        void acceptsAZeroUptiltBecauseThatIsTheDefaultAndMustNotBeRejectedAsNonPositive() {
            assertEquals(0.0f, tuning(0.2f, 0.2f, 0.0f).cameraUptiltRadians(), EPSILON);
        }

        @Test
        void acceptsANegativeUptiltBecauseADownwardCameraTiltIsMeaningful() {
            assertEquals(-0.3f, tuning(0.2f, 0.2f, -0.3f).cameraUptiltRadians(), EPSILON);
        }
    }

    @Nested
    class Defaults {

        @Test
        void startsInTrackedModeBecauseThatIsTheModeTheTicketAsksFor() {
            assertSame(DroneCameraView.TRACKED, DroneCameraTuning.DEFAULT.view());
        }

        @Test
        void usesHytalesOwnLerpSpeedRatherThanTheProtocolDefault() {
            // The protocol defaults both to 1.0; all three in-tree camera commands use 0.2.
            assertEquals(0.2f, DroneCameraTuning.DEFAULT_LERP_SPEED, EPSILON);
            assertEquals(0.2f, DroneCameraTuning.DEFAULT.positionLerpSpeed(), EPSILON);
            assertEquals(0.2f, DroneCameraTuning.DEFAULT.rotationLerpSpeed(), EPSILON);
        }

        @Test
        void startsWithZeroUptiltSoTheFirstFlightTestCannotMistakeItForAPitchSignError() {
            assertEquals(0.0f, DroneCameraTuning.DEFAULT.cameraUptiltRadians(), EPSILON);
        }

        @Test
        void matchesTheEyeHeightAuthoredIntoTheDroneModel() {
            assertEquals(0.1, DroneCameraTuning.DEFAULT_EYE_HEIGHT, EPSILON);
            assertEquals(0.1, DroneCameraTuning.DEFAULT.eyeHeight(), EPSILON);
        }

        @Test
        void startsLockedBecauseAllThreeInTreeCameraCommandsLock() {
            assertTrue(DroneCameraTuning.DEFAULT.locked());
        }
    }

    @Nested
    class Withers {

        @Test
        void eachWitherChangesOnlyItsOwnFieldSoRuntimeTuningCannotSilentlyResetAnother() {
            DroneCameraTuning base = new DroneCameraTuning(
                    DroneCameraView.TRACKED, 0.3f, 0.4f, true, 0.15, 0.25f);

            DroneCameraTuning viewChanged = base.withView(DroneCameraView.DRIVEN);
            assertSame(DroneCameraView.DRIVEN, viewChanged.view());
            assertEquals(0.3f, viewChanged.positionLerpSpeed(), EPSILON);
            assertEquals(0.4f, viewChanged.rotationLerpSpeed(), EPSILON);
            assertTrue(viewChanged.locked());
            assertEquals(0.15, viewChanged.eyeHeight(), EPSILON);
            assertEquals(0.25f, viewChanged.cameraUptiltRadians(), EPSILON);

            DroneCameraTuning lerpChanged = base.withLerpSpeeds(0.7f, 0.8f);
            assertEquals(0.7f, lerpChanged.positionLerpSpeed(), EPSILON);
            assertEquals(0.8f, lerpChanged.rotationLerpSpeed(), EPSILON);
            assertSame(DroneCameraView.TRACKED, lerpChanged.view());
            assertEquals(0.25f, lerpChanged.cameraUptiltRadians(), EPSILON);

            assertEquals(0.9f, base.withCameraUptiltRadians(0.9f).cameraUptiltRadians(), EPSILON);
            assertEquals(0.25f, base.withLocked(false).cameraUptiltRadians(), EPSILON);
        }

        @Test
        void withersStillValidateSoARuntimeCommandCannotInstallANaN() {
            assertThrows(IllegalArgumentException.class,
                    () -> DroneCameraTuning.DEFAULT.withLerpSpeeds(Float.NaN, 0.2f));
            assertThrows(IllegalArgumentException.class,
                    () -> DroneCameraTuning.DEFAULT.withCameraUptiltRadians(Float.NaN));
        }
    }
}
