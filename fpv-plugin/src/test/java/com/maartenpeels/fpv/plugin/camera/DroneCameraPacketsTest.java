package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.protocol.ApplyLookType;
import com.hypixel.hytale.protocol.AttachedToType;
import com.hypixel.hytale.protocol.ClientCameraView;
import com.hypixel.hytale.protocol.Direction;
import com.hypixel.hytale.protocol.MovementForceRotationType;
import com.hypixel.hytale.protocol.PositionType;
import com.hypixel.hytale.protocol.RotationType;
import com.hypixel.hytale.protocol.ServerCameraSettings;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import org.joml.Vector3d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The only part of #19 that can be asserted mechanically: the exact contents of packet 280.
 *
 * <p>Whether the client <em>renders</em> any of this needs a human. What these tests protect is
 * everything upstream of that — the field values, the two view modes, the unit of the angles, and
 * the three fields other tickets silently depend on.
 */
class DroneCameraPacketsTest {

    private static final float EPSILON = 1.0e-6f;

    /** pitch, yaw, roll — all different, all distinguishable if two get swapped. */
    private static final Rotation3f ATTITUDE = new Rotation3f(0.11f, 0.22f, 0.33f);
    private static final Vector3d POSITION = new Vector3d(10.0, 20.0, 30.0);

    private static ServerCameraSettings settingsFor(DroneCameraTuning tuning) {
        SetServerCamera packet = DroneCameraPackets.attach(tuning, 4242, ATTITUDE, POSITION);
        assertNotNull(packet.cameraSettings, "attach must always carry settings");
        return packet.cameraSettings;
    }

    private static DroneCameraTuning tracked() {
        return DroneCameraTuning.DEFAULT.withView(DroneCameraView.TRACKED);
    }

    private static DroneCameraTuning driven() {
        return DroneCameraTuning.DEFAULT.withView(DroneCameraView.DRIVEN);
    }

    @Nested
    class Detach {

        @Test
        void releasesWithANullSettingsBlockBecauseThatIsHowTheServerItselfReleasesACamera() {
            // Byte-identical to PlayerCameraResetCommand.java:33 and CameraManager.java:40. The
            // surprising part is that release is NOT ClientCameraView.FirstPerson.
            SetServerCamera packet = DroneCameraPackets.detach();

            assertEquals(ClientCameraView.Custom, packet.clientCameraView);
            assertFalse(packet.isLocked);
            assertNull(packet.cameraSettings);
        }

        @Test
        void isTheDoneWhenOfTheTicketSoItMustNeverCarrySettings() {
            // A release that carried settings would leave the pilot in *some* server camera, which
            // is exactly the "stuck in a weird view" failure the ticket's Done-when rules out.
            assertNull(DroneCameraPackets.detach().cameraSettings);
        }
    }

    @Nested
    class SharedFields {

        @Test
        void alwaysUsesCustomViewBecauseNoOtherClientCameraViewIsEverSentByTheServer() {
            for (DroneCameraView view : DroneCameraView.values()) {
                SetServerCamera packet = DroneCameraPackets.attach(
                        DroneCameraTuning.DEFAULT.withView(view), 1, ATTITUDE, POSITION);
                assertEquals(ClientCameraView.Custom, packet.clientCameraView, view.name());
            }
        }

        @Test
        void isFirstPersonBecauseTheTicketIsFpvNotAChaseCam() {
            for (DroneCameraView view : DroneCameraView.values()) {
                assertTrue(settingsFor(DroneCameraTuning.DEFAULT.withView(view)).isFirstPerson,
                        view.name());
            }
        }

        @Test
        void skipsCharacterPhysicsBecauseDecisionFourParksThePilotsBody() {
            // Also what #20's body-follow inherits: the body is teleported to the drone every tick
            // purely to drag chunk streaming along, and this is what stops its character controller
            // fighting that.
            for (DroneCameraView view : DroneCameraView.values()) {
                assertTrue(settingsFor(DroneCameraTuning.DEFAULT.withView(view)).skipCharacterPhysics,
                        view.name());
            }
        }

        @Test
        void allowsPitchControlsBecauseTheProtocolDefaultIsFalseAndIssue17DiesWithoutIt() {
            // The single most valuable assertion in this file. ServerCameraSettings.java:24 defaults
            // allowPitchControls to false, and docs/plans/17.md:109 records that the input adapter's
            // pitch channel goes dead without it. Nothing about a dead pitch channel looks like a
            // camera bug, so this would be a very expensive regression to find by flying.
            for (DroneCameraView view : DroneCameraView.values()) {
                assertTrue(settingsFor(DroneCameraTuning.DEFAULT.withView(view)).allowPitchControls,
                        view.name());
            }
        }

        @Test
        void leavesApplyLookTypeAtLocalPlayerBecauseThatIsTheChannelIssue17Reads() {
            // ApplyLookType.Rotation would kill it (docs/plans/17.md:106-108).
            for (DroneCameraView view : DroneCameraView.values()) {
                assertEquals(
                        ApplyLookType.LocalPlayerLookOrientation,
                        settingsFor(DroneCameraTuning.DEFAULT.withView(view)).applyLookType,
                        view.name());
            }
        }

        @Test
        void leavesMovementForceRotationAtAttachedToHeadBecauseThatIsTheFrameIssue17Assumes() {
            // The one-arg ClientMovementAdapter.sample(packet) treats wishMovement as being in the
            // pilot's head frame. Changing this silently invalidates that.
            for (DroneCameraView view : DroneCameraView.values()) {
                assertEquals(
                        MovementForceRotationType.AttachedToHead,
                        settingsFor(DroneCameraTuning.DEFAULT.withView(view)).movementForceRotationType,
                        view.name());
            }
        }

        @Test
        void disablesEyeOffsetBecauseWeSupplyOurOwnAndTwoWouldDoubleUp() {
            // Every in-tree sender writes true, and the model that #18 authored carries
            // EyeHeight: 0.1 -- so there is a real value here to double.
            for (DroneCameraView view : DroneCameraView.values()) {
                assertFalse(settingsFor(DroneCameraTuning.DEFAULT.withView(view)).eyeOffset,
                        view.name());
            }
        }

        @Test
        void hidesTheReticleBecauseADroneFeedHasNoCrosshair() {
            for (DroneCameraView view : DroneCameraView.values()) {
                assertFalse(settingsFor(DroneCameraTuning.DEFAULT.withView(view)).displayReticle,
                        view.name());
            }
        }

        @Test
        void carriesTheLerpSpeedsVerbatimBecauseTheyAreTheSmoothingLeverIssue24NeedsToTry() {
            ServerCameraSettings settings =
                    settingsFor(DroneCameraTuning.DEFAULT.withLerpSpeeds(0.35f, 0.45f));

            assertEquals(0.35f, settings.positionLerpSpeed, EPSILON);
            assertEquals(0.45f, settings.rotationLerpSpeed, EPSILON);
        }

        @Test
        void carriesTheLockFlagOnThePacketNotTheSettings() {
            assertTrue(DroneCameraPackets
                    .attach(DroneCameraTuning.DEFAULT.withLocked(true), 1, ATTITUDE, POSITION)
                    .isLocked);
            assertFalse(DroneCameraPackets
                    .attach(DroneCameraTuning.DEFAULT.withLocked(false), 1, ATTITUDE, POSITION)
                    .isLocked);
        }
    }

    @Nested
    class Tracked {

        @Test
        void attachesToTheDronesNetworkIdSoTheClientCanFollowItItself() {
            ServerCameraSettings settings =
                    DroneCameraPackets.attach(tracked(), 4242, ATTITUDE, POSITION).cameraSettings;

            assertNotNull(settings);
            assertEquals(AttachedToType.EntityId, settings.attachedToType);
            assertEquals(4242, settings.attachedToEntityId);
        }

        @Test
        void usesOffsetModeForBothPositionAndRotationSoTheDronesOwnTransformDrivesTheCamera() {
            ServerCameraSettings settings = settingsFor(tracked());

            assertEquals(PositionType.AttachedToPlusOffset, settings.positionType);
            assertEquals(RotationType.AttachedToPlusOffset, settings.rotationType);
        }

        @Test
        void sendsNoAbsolutePositionOrRotationBecauseOffsetModeWouldIgnoreThem() {
            // Sending both an attachment and an absolute value invites the client to blend two
            // sources, and would make a failure impossible to attribute to either mode.
            ServerCameraSettings settings = settingsFor(tracked());

            assertNull(settings.position);
            assertNull(settings.rotation);
        }

        @Test
        void putsEyeHeightInThePositionOffsetSoTheViewSitsOnTheAirframe() {
            ServerCameraSettings settings = settingsFor(tracked());

            assertNotNull(settings.positionOffset);
            assertEquals(0.0, settings.positionOffset.x, EPSILON);
            assertEquals(DroneCameraTuning.DEFAULT_EYE_HEIGHT, settings.positionOffset.y, EPSILON);
            assertEquals(0.0, settings.positionOffset.z, EPSILON);
        }

        @Test
        void putsCameraUptiltInTheRotationOffsetPitchAndNowhereElse() {
            ServerCameraSettings settings =
                    settingsFor(tracked().withCameraUptiltRadians(0.5f));

            assertNotNull(settings.rotationOffset);
            assertEquals(0.0f, settings.rotationOffset.yaw, EPSILON);
            assertEquals(0.5f, settings.rotationOffset.pitch, EPSILON);
            assertEquals(0.0f, settings.rotationOffset.roll, EPSILON);
        }
    }

    @Nested
    class Driven {

        @Test
        void attachesToNothingBecauseTheServerSuppliesPositionAndRotationItself() {
            // This is what makes DRIVEN independent of AttachedToType.EntityId -- the enum value
            // with zero usages anywhere in the decompiled server, and the largest unknown in #19.
            assertEquals(AttachedToType.None, settingsFor(driven()).attachedToType);
        }

        @Test
        void usesCustomModeForBothPositionAndRotation() {
            ServerCameraSettings settings = settingsFor(driven());

            assertEquals(PositionType.Custom, settings.positionType);
            assertEquals(RotationType.Custom, settings.rotationType);
        }

        @Test
        void sendsNoOffsetsBecauseCustomModeFoldsThemIntoTheAbsoluteValues() {
            ServerCameraSettings settings = settingsFor(driven());

            assertNull(settings.positionOffset);
            assertNull(settings.rotationOffset);
        }

        @Test
        void sendsTheDronePositionWithEyeHeightAddedToYOnly() {
            ServerCameraSettings settings = settingsFor(driven());

            assertNotNull(settings.position);
            assertEquals(10.0, settings.position.x, EPSILON);
            assertEquals(20.0 + DroneCameraTuning.DEFAULT_EYE_HEIGHT, settings.position.y, EPSILON);
            assertEquals(30.0, settings.position.z, EPSILON);
        }

        @Test
        void sendsTheFullThreeAxisAttitudeIncludingRollBecauseRollIsTheWholePointOfTheTicket() {
            ServerCameraSettings settings = settingsFor(driven());

            assertNotNull(settings.rotation);
            assertEquals(0.33f, settings.rotation.roll, EPSILON);
        }

        @Test
        void doesNotTransposeYawAndPitchWhenConvertingRotation3fToDirection() {
            // Rotation3f is (pitch, yaw, roll); Direction is (yaw, pitch, roll). The first two
            // arguments swap, and a swap is invisible in flight -- it looks like a physics bug.
            // ATTITUDE is pitch=0.11, yaw=0.22, so a swap makes this fail on both assertions.
            ServerCameraSettings settings = settingsFor(driven());

            assertNotNull(settings.rotation);
            assertEquals(0.22f, settings.rotation.yaw, EPSILON, "yaw must come from Rotation3f.yaw()");
            assertEquals(0.11f, settings.rotation.pitch, EPSILON, "pitch must come from Rotation3f.pitch()");
        }

        @Test
        void doesNotNegatePitchBecauseTheAttitudeIsAlreadyInHytalesConvention() {
            // CLAUDE.md is right that Hytale's Direction.pitch is positive nose-up, opposite to
            // ControlInput.pitch. But TransformComponent.getRotation() is already Hytale-native --
            // PositionUtil.assign copies it into a Direction with no negation. The ControlInput
            // negation belongs where the sim writes attitude into the transform, which is #24.
            // Negating here too would double-negate and tilt the horizon the wrong way.
            ServerCameraSettings settings =
                    settingsFor(driven().withCameraUptiltRadians(0.0f));

            assertNotNull(settings.rotation);
            assertEquals(ATTITUDE.pitch(), settings.rotation.pitch, EPSILON);
        }

        @Test
        void addsCameraUptiltToPitchBecauseHytalePitchIsPositiveNoseUp() {
            ServerCameraSettings settings = settingsFor(driven().withCameraUptiltRadians(0.25f));

            assertNotNull(settings.rotation);
            assertEquals(0.11f + 0.25f, settings.rotation.pitch, EPSILON);
            // Uptilt must not leak into the other two axes.
            assertEquals(0.22f, settings.rotation.yaw, EPSILON);
            assertEquals(0.33f, settings.rotation.roll, EPSILON);
        }
    }

    @Nested
    class Radians {

        @Test
        void treatsAnglesAsRadiansWithNoUnitConversionAnywhere() {
            // Direction is radians throughout the protocol -- PlayerCameraTopdownCommand.java:50
            // passes -PI/2 to look straight down. A degrees value reaching the wire is ~5 rotations
            // and renders as garbage that reads as "the client ignores this field". That exact false
            // negative nearly killed #28 for the wrong reason.
            Rotation3f quarterTurnRoll = new Rotation3f(0.0f, 0.0f, (float) (Math.PI / 2.0));

            Direction direction = DroneCameraPackets.toDirection(quarterTurnRoll, 0.0f);

            assertEquals(Math.PI / 2.0, direction.roll, EPSILON);
        }

        @Test
        void passesARadianRollStraightThroughToThePacket() {
            Rotation3f rolled = new Rotation3f(0.0f, 0.0f, (float) Math.toRadians(30.0));

            ServerCameraSettings settings =
                    DroneCameraPackets.attach(driven(), 1, rolled, POSITION).cameraSettings;

            assertNotNull(settings);
            assertNotNull(settings.rotation);
            assertEquals(Math.toRadians(30.0), settings.rotation.roll, EPSILON);
        }
    }
}
