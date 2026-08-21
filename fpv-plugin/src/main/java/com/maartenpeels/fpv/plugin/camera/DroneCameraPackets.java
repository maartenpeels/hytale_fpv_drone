package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.math.vector.Rotation3fc;
import com.hypixel.hytale.protocol.AttachedToType;
import com.hypixel.hytale.protocol.ClientCameraView;
import com.hypixel.hytale.protocol.Direction;
import com.hypixel.hytale.protocol.Position;
import com.hypixel.hytale.protocol.PositionType;
import com.hypixel.hytale.protocol.ServerCameraSettings;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import org.joml.Vector3dc;

import javax.annotation.Nonnull;

/**
 * Builds every {@code SetServerCamera} (packet 280) this plugin sends. Pure: arguments in, packet
 * out, no store, no world, no network.
 *
 * <p><strong>This class exists because #19 is otherwise unverifiable.</strong> The server keeps no
 * record of what it told a client the camera should follow — {@code CameraManager}
 * (`server/core/entity/entities/player/CameraManager.java:19-24`) holds mouse state and the last
 * targeted block, nothing about attachment. There is no "read back the camera" query. So the only
 * thing about this feature that can be asserted mechanically is the content of the packet, and
 * keeping construction in one side-effect-free place is what makes even that much testable. Per
 * CLAUDE.md's ECS conventions: computation here, side effects in {@link DroneCamera}.
 *
 * <h2>Two conversions that are easy to get wrong and invisible when wrong</h2>
 *
 * <p><strong>{@code Direction} is radians.</strong> {@code PlayerCameraTopdownCommand.java:50}
 * passes {@code -Math.PI / 2} to look straight down. A value in degrees is roughly five full
 * rotations and renders as garbage that reads as "the client ignores this field" — the false
 * negative that nearly killed #28 for the wrong reason. Nothing in this class takes degrees.
 *
 * <p><strong>The angle triples are transposed.</strong> {@code Rotation3f} is
 * {@code (pitch, yaw, roll)} — its fields are literally {@code x, y, z}
 * (`math/vector/Rotation3f.java:33-35`, with {@code x} commented as pitch). {@code Direction}'s
 * constructor is {@code (yaw, pitch, roll)} (`protocol/Direction.java:24`). The first two arguments
 * swap. {@link #toDirection} is the single conversion point, and
 * {@code DroneCameraPacketsTest} asserts yaw and pitch separately so a swap cannot pass.
 *
 * <h2>No sign flip here</h2>
 *
 * <p>CLAUDE.md is right that Hytale's {@code Direction.pitch} is positive nose-<em>up</em>, opposite
 * to {@code ControlInput.pitch}'s transmitter convention. But the attitude reaching this class comes
 * from {@code TransformComponent.getRotation()}, which is <em>already</em> Hytale-native: it is the
 * same value {@code PositionUtil.assign} copies straight into a {@code Direction} with no negation
 * (`server/core/util/PositionUtil.java:58-62`). The {@code ControlInput} → Hytale negation belongs
 * where the sim writes attitude <em>into</em> {@code TransformComponent}, which is #24. Negating
 * here as well would double-negate and tilt the horizon the wrong way — silently.
 */
public final class DroneCameraPackets {

    private DroneCameraPackets() {}

    /**
     * The packet that returns a pilot to a normal view.
     *
     * <p>Byte-identical to how the server itself releases a camera, in both places it does so:
     * {@code PlayerCameraResetCommand.java:33} and {@code CameraManager.java:40}. The surprising
     * part, and the reason this is a named method rather than an inline constructor call, is that
     * <strong>release is not {@code ClientCameraView.FirstPerson}</strong> — it keeps {@code Custom}
     * and relies on {@code cameraSettings == null} with {@code isLocked == false}. Neither
     * {@code FirstPerson} nor {@code ThirdPerson} is sent anywhere in the server tree.
     */
    @Nonnull
    public static SetServerCamera detach() {
        return new SetServerCamera(ClientCameraView.Custom, false, null);
    }

    /**
     * The packet that puts a pilot in the drone.
     *
     * @param tuning view mode and the smoothing/lock/offset knobs
     * @param droneNetworkId the drone's {@code NetworkId}, used only by
     *     {@link DroneCameraView#TRACKED}. Must be the network id, never {@code Ref.getIndex()} —
     *     see {@link DroneCamera}.
     * @param attitude the drone's current rotation, in Hytale's convention, already native
     * @param position the drone's current position
     */
    @Nonnull
    public static SetServerCamera attach(
            @Nonnull DroneCameraTuning tuning,
            int droneNetworkId,
            @Nonnull Rotation3fc attitude,
            @Nonnull Vector3dc position) {

        ServerCameraSettings settings = new ServerCameraSettings();

        settings.positionLerpSpeed = tuning.positionLerpSpeed();
        settings.rotationLerpSpeed = tuning.rotationLerpSpeed();

        // The ticket: FPV, not a chase cam.
        settings.isFirstPerson = true;

        // Decision 4. Stops the parked body's character physics from fighting the sim, and it is
        // what #20's body-follow inherits -- see DroneCamera.
        settings.skipCharacterPhysics = true;

        // NOT the protocol default, which is false (`protocol/ServerCameraSettings.java:24`).
        // docs/plans/17.md:109: the pitch channel #17's input adapter reads goes dead without it.
        settings.allowPitchControls = true;

        // A drone feed has no crosshair.
        settings.displayReticle = false;

        // Explicit, and explicitly false. The server never reads this flag -- it is client-side
        // only -- but all three in-tree senders write true, and the circumstantial evidence
        // (`Model.java:178` shipping the model's EyeHeight to the client; `TargetUtil.java:388-398`
        // applying eye height as a +Y offset) is that it silently adds the attached model's
        // EyeHeight to the camera Y. We apply our own offset from DroneCameraTuning, so leaving
        // this true would double up -- and #18 authored EyeHeight: 0.1 into the drone model, so
        // there is a real value there to double.
        settings.eyeOffset = false;

        // Deliberately untouched, and load-bearing for #17:
        //   applyLookType stays LocalPlayerLookOrientation -- the channel the input adapter reads;
        //     ApplyLookType.Rotation kills it (docs/plans/17.md:106-108).
        //   movementForceRotationType stays AttachedToHead -- the frame the one-arg
        //     ClientMovementAdapter.sample(packet) assumes for wishMovement.
        //   canMoveType stays AttachedToLocalPlayer -- semantics unknown, no in-tree precedent.
        //   sendMouseMotion stays false -- the packet-111 channel #17 recorded as a fallback.

        settings.rotationType = tuning.view().rotationType();

        switch (tuning.view()) {
            case TRACKED -> {
                settings.attachedToType = AttachedToType.EntityId;
                settings.attachedToEntityId = droneNetworkId;
                settings.positionType = PositionType.AttachedToPlusOffset;
                settings.positionOffset = new Position(0.0, tuning.eyeHeight(), 0.0);
                settings.rotationOffset = new Direction(0.0f, tuning.cameraUptiltRadians(), 0.0f);
            }
            case DRIVEN -> {
                // Nothing to attach to: the server supplies absolute position and rotation every
                // tick, so naming an entity would only invite the client to blend two sources.
                settings.attachedToType = AttachedToType.None;
                settings.positionType = PositionType.Custom;
                settings.position =
                        new Position(position.x(), position.y() + tuning.eyeHeight(), position.z());
                settings.rotation = toDirection(attitude, tuning.cameraUptiltRadians());
            }
        }

        // Custom is what every in-tree sender uses; FirstPerson/ThirdPerson are never sent. The
        // first-person-ness is carried by settings.isFirstPerson, not by this enum.
        return new SetServerCamera(ClientCameraView.Custom, tuning.locked(), settings);
    }

    /**
     * {@code Rotation3fc} → {@code Direction}, with a camera tilt folded into pitch.
     *
     * <p>The one place the {@code (pitch, yaw, roll)} / {@code (yaw, pitch, roll)} transposition
     * happens. Uptilt adds to pitch because Hytale's pitch is positive nose-up, so tilting the
     * camera up is a positive addition.
     */
    @Nonnull
    static Direction toDirection(@Nonnull Rotation3fc attitude, float uptiltRadians) {
        return new Direction(attitude.yaw(), attitude.pitch() + uptiltRadians, attitude.roll());
    }
}
