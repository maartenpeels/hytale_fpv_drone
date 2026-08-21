package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.math.vector.Rotation3fc;
import com.maartenpeels.fpv.math.Quat;
import org.joml.Quaterniond;
import org.joml.Vector3d;

import javax.annotation.Nonnull;

/**
 * Converts between {@code :fpv-core}'s attitude quaternion and Hytale's {@code Rotation3f}.
 *
 * <p>{@code Quat}'s javadoc says it deliberately has no Euler readout, because Hytale's angle
 * conventions are the plugin's problem and belong where they can be tested against the real types.
 * This is that place.
 *
 * <h2>No sign flips, and why that is not a mistake</h2>
 *
 * CLAUDE.md warns that "{@code Direction}'s signs disagree with {@code ControlInput}'s, on two axes out
 * of three" and that a rotation adapter "must negate" pitch. That warning is about
 * {@link com.maartenpeels.fpv.control.ControlInput}, which is <em>transmitter</em> convention —
 * positive pitch is nose-down, positive yaw is nose-right, because that is what a stick does.
 * {@code Rotation3f} and {@code Quat} are both <em>geometric</em>, and both right-handed, so between
 * those two nothing is negated. Applying the transmitter rule here would invert the drone's attitude
 * on screen relative to its simulated one, which reads as "the model is wrong" rather than as a sign
 * error.
 *
 * <p>Every step of that verified against the decompiled server:
 *
 * <ul>
 *   <li>{@code Rotation3f.getQuaternion} is {@code dest.rotationYXZ(yaw, pitch, roll)} and the inverse
 *       readout used by {@code premul}/{@code mul} is {@code q.getEulerAnglesYXZ(...)}
 *       (`math/vector/Rotation3f.java:235-252`) — JOML, right-handed. Corroborated by
 *       {@code Rotation3f.transform}, which is {@code rotateZ(roll).rotateX(pitch).rotateY(yaw)}
 *       (`:263-266`).
 *   <li>{@code Rotation3f} maps 1:1 onto the wire: {@code PositionUtil.toDirectionPacket} is
 *       {@code new Direction(rotation.yaw(), rotation.pitch(), rotation.roll())}
 *       (`server/core/util/PositionUtil.java:26-28`).
 *   <li>Hytale yaw <b>is</b> a right-handed rotation about {@code +Y}: {@code Ry(θ)} applied to
 *       {@code (0,0,−1)} gives {@code (−sinθ, 0, −cosθ)}, which is exactly
 *       {@code Vector3dUtil.setYawPitch}'s {@code x = −sin(yaw)·cos(pitch)},
 *       {@code z = −cos(yaw)·cos(pitch)}. CLAUDE.md's "opposite to a right-handed yaw about +Y" is
 *       wrong; #17's plan established this and the correction is repeated in {@code docs/plans/23.md}.
 *   <li>Core's body axes are {@code +X} right, {@code +Y} up, {@code −Z} forward
 *       ({@code DroneState}'s javadoc), and a right-handed rotation about {@code +X} by φ takes forward
 *       to {@code (0, sinφ, −cosφ)} — nose <em>up</em> for positive φ, matching Hytale's
 *       positive-nose-up pitch.
 * </ul>
 *
 * <h2>Two traps in the types</h2>
 *
 * {@code Rotation3f}'s constructor is {@code (pitch, yaw, roll)} while its accessors are named
 * {@code yaw()}/{@code pitch()}/{@code roll()}, and JOML's {@code getEulerAnglesYXZ} answers a vector
 * whose {@code x} is the pitch and {@code y} the yaw. Both orderings are easy to transpose and neither
 * transposition fails to compile.
 *
 * <h2>Gimbal lock is inherent, not a defect here</h2>
 *
 * A drone hovering nose-down sits at the ±90° pitch singularity, where yaw and roll stop being
 * separable and the readout jumps. Nothing can avoid that: the wire format is three angles. It affects
 * only what the client renders — the simulation stays quaternion-based end to end — so the worst case
 * is a drone that looks like it is spinning while flying perfectly.
 */
public final class DroneRotation {

    private DroneRotation() {}

    /** Hytale's angles for a core attitude quaternion. Allocates; call once per tick per drone. */
    @Nonnull
    public static Rotation3f toRotation(@Nonnull Quat orientation) {
        Rotation3f rotation = new Rotation3f();
        writeTo(orientation, rotation);
        return rotation;
    }

    /** As {@link #toRotation}, writing into an existing {@code Rotation3f} rather than allocating. */
    public static void writeTo(@Nonnull Quat orientation, @Nonnull Rotation3f into) {
        // JOML's constructor is (x, y, z, w); Quat's component order is (w, x, y, z).
        Quaterniond joml =
                new Quaterniond(orientation.x(), orientation.y(), orientation.z(), orientation.w());
        Vector3d euler = joml.getEulerAnglesYXZ(new Vector3d());
        // (x, y, z) here is (pitch, yaw, roll) -- the order Rotation3f.set(float,float,float) wants,
        // and the order Rotation3f's own premul/mul use when reading a quaternion back.
        into.set((float) euler.x, (float) euler.y, (float) euler.z);
    }

    /**
     * The core attitude for Hytale's angles — the inverse of {@link #toRotation}.
     *
     * <p>Used to seed a freshly spawned drone from the transform {@code FlightSessions.launch} gave it,
     * so the drone starts facing where its pilot faced instead of at yaw zero.
     */
    @Nonnull
    public static Quat toQuat(@Nonnull Rotation3fc rotation) {
        Quaterniond joml =
                new Quaterniond().rotationYXZ(rotation.yaw(), rotation.pitch(), rotation.roll());
        return new Quat(joml.w, joml.x, joml.y, joml.z);
    }

    /**
     * The core attitude for a heading alone — {@code rotation}'s yaw, with pitch and roll dropped.
     *
     * <p>What a drone arms at. A quad that materialised already banked would be a surprise, and the
     * pilot's own head pitch has nothing to do with the airframe's.
     */
    @Nonnull
    public static Quat headingOf(@Nonnull Rotation3fc rotation) {
        Quaterniond joml = new Quaterniond().rotationYXZ(rotation.yaw(), 0.0, 0.0);
        return new Quat(joml.w, joml.x, joml.y, joml.z);
    }
}
