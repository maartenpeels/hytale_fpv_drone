package com.maartenpeels.fpv.plugin.input;

import com.hypixel.hytale.protocol.Direction;
import com.hypixel.hytale.protocol.Position;
import com.hypixel.hytale.protocol.packets.player.ClientMovement;
import com.maartenpeels.fpv.control.PilotInputSample;

/**
 * The whole of the Hytale side of decision 1's boundary: {@code ClientMovement} in,
 * {@link PilotInputSample} out.
 *
 * <p>Field copies and null checks, and deliberately nothing else — no arithmetic, no unit
 * conversion, not even the sign flips. Those all live in {@code PilotInputMapper} in
 * {@code :fpv-core}, where they are under unit test with no server on the classpath. Decision 1 bets
 * that this class is the <em>only</em> thing that changes when the wire protocol does, and the way to
 * make that bet pay is to keep it boring enough that a protocol change cannot hide a behaviour change
 * inside it. <b>If you find yourself wanting to compute something here, it belongs in the core
 * mapper.</b>
 *
 * <p>Every field of {@code ClientMovement} is {@code @Nullable} and the client omits what has not
 * changed, so absence is routine. A missing {@code wishMovement} reads as a centred left stick; a
 * missing {@code lookOrientation} becomes {@code NaN}, which {@link PilotInputSample} defines as
 * "this packet carried no look angle" and the mapper answers with zero pitch/roll while keeping its
 * memory of the last real angle.
 *
 * <p>Three things this does not read, each for a reason:
 *
 * <ul>
 *   <li>{@code lookOrientation.roll} — nothing a pilot can touch produces camera roll, on a mouse or
 *       a gamepad. Roll is derived from the look-<em>yaw</em> channel instead; see
 *       {@code PilotInputMapper}.
 *   <li>{@code wishMovement.y} — vertical intent in Hytale's character controller rides the
 *       {@code jumping}/{@code crouching} booleans in {@code MovementStates}, not the wish vector,
 *       and throttle is already spoken for by the forward axis.
 *   <li>{@code movementStates} — 23 booleans with no analog channel among them. Arm/disarm and mode
 *       switches will want some of them eventually; flight axes never will.
 * </ul>
 */
public final class ClientMovementAdapter {

    private ClientMovementAdapter() {}

    /**
     * Reads a packet whose {@code wishMovement} is in the pilot's own look frame — Hytale's default,
     * since {@code ServerCameraSettings.movementForceRotationType} defaults to
     * {@code AttachedToHead} and the client rotates the stick vector by the head rotation before
     * sending it.
     *
     * <p>If the packet carried no {@code lookOrientation} there is no head yaw to name the frame
     * with, so the frame is left unknown and the core mapper falls back to the yaw it last saw.
     */
    public static PilotInputSample sample(ClientMovement packet) {
        if (packet == null || packet.lookOrientation == null) {
            return sample(packet, Double.NaN);
        }
        return sample(packet, packet.lookOrientation.yaw);
    }

    /**
     * Reads a packet whose {@code wishMovement} frame the caller knows independently — the case when
     * the camera pins it with {@code MovementForceRotationType.Custom} and an explicit
     * {@code movementForceRotation}, where the frame is that rotation's yaw rather than the pilot's
     * head yaw.
     *
     * @param wishFrameYaw the yaw {@code wishMovement} was rotated into, in radians; {@code NaN} to
     *     leave it unknown
     */
    public static PilotInputSample sample(ClientMovement packet, double wishFrameYaw) {
        if (packet == null) {
            return PilotInputSample.EMPTY;
        }
        Position wish = packet.wishMovement;
        Direction look = packet.lookOrientation;
        return new PilotInputSample(
                wish == null ? 0.0 : wish.x,
                wish == null ? 0.0 : wish.z,
                wishFrameYaw,
                look == null ? Double.NaN : look.yaw,
                look == null ? Double.NaN : look.pitch);
    }
}
