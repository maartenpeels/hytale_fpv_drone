package com.maartenpeels.fpv.flight;

import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;

/**
 * A drone's complete kinematic state: where it is, how fast it is going, which way it is pointing,
 * and how fast it is rotating.
 *
 * <p>This is the whole of what {@link QuadIntegrator} reads and writes. Nothing else is carried —
 * no accumulated PID error, no timestamps, no identity — so a step is a pure function of
 * {@code (state, input, dt)} and the same three arguments always produce the same fourth.
 *
 * <h2>Frame conventions</h2>
 *
 * The single place these are written down. Everything downstream inherits them.
 *
 * <ul>
 *   <li><b>World</b>: right-handed, {@code +Y} up, matching Hytale (verified — its
 *       {@code ForceProviderStandard} applies negated gravity to {@code force.y}). Gravity is
 *       {@code (0, −g, 0)}.
 *   <li><b>Body axes at identity orientation coincide with the world axes</b>: {@code +X} right,
 *       {@code +Y} up and along the thrust vector, {@code −Z} forward. Forward being {@code −Z}
 *       matches Hytale's yaw-0 heading, which keeps the camera adapter to a sign flip rather than a
 *       180° offset.
 *   <li><b>Rotation</b> is a {@link Quat}, not Euler angles, because a quad rolls and loops through
 *       vertical where Euler angles gimbal-lock.
 *   <li><b>Angular velocity</b> is a {@link BodyRates}, in the pilot's sign convention rather than
 *       raw axes. See {@link BodyRates#toBodyAxes()}.
 * </ul>
 *
 * <p>Positions are {@code double} throughout; Hytale's {@code protocol.Position} is too, so nothing
 * is lost when the plugin adapter hands one over.
 */
public record DroneState(Vec3 position, Vec3 velocity, Quat orientation, BodyRates bodyRates) {

    public DroneState {
        requireFinite(position, "position");
        requireFinite(velocity, "velocity");
        if (orientation == null) {
            throw new IllegalArgumentException("orientation must not be null");
        }
        if (bodyRates == null || !bodyRates.isFinite()) {
            throw new IllegalArgumentException("bodyRates must be finite but was " + bodyRates);
        }
    }

    /** A level, stationary drone at {@code position} — how one starts life at {@code /fpv launch}. */
    public static DroneState restingAt(Vec3 position) {
        return new DroneState(position, Vec3.ZERO, Quat.IDENTITY, BodyRates.ZERO);
    }

    /** The direction thrust acts in, in world space: the body's up axis, rotated. */
    public Vec3 thrustAxis() {
        return this.orientation.rotate(Vec3.UP);
    }

    /** Where the nose points, in world space. */
    public Vec3 forward() {
        return this.orientation.rotate(new Vec3(0, 0, -1));
    }

    /** The body's right-hand side, in world space. */
    public Vec3 right() {
        return this.orientation.rotate(new Vec3(1, 0, 0));
    }

    public double speed() {
        return this.velocity.length();
    }

    private static void requireFinite(Vec3 value, String name) {
        if (value == null || !value.isFinite()) {
            throw new IllegalArgumentException(name + " must be finite but was " + value);
        }
    }
}
