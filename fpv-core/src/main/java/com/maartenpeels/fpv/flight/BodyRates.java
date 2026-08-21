package com.maartenpeels.fpv.flight;

import com.maartenpeels.fpv.math.SignedZero;
import com.maartenpeels.fpv.math.Vec3;

/**
 * Angular velocity about the drone's own axes, in radians per second, expressed in the pilot's
 * sign convention rather than as a raw axis vector.
 *
 * <p>The signs are {@link com.maartenpeels.fpv.control.ControlInput}'s, so a stick position and the
 * rate it demands read the same way: positive {@code roll} banks right, positive {@code pitch} is
 * nose <strong>down</strong>, positive {@code yaw} is nose right.
 *
 * <p>That convention is transmitter-facing, while the flight model's frame is view-facing
 * ({@code +X} right, {@code +Y} up, {@code −Z} forward). The two disagree on all three axes, so the
 * conversion to an axis vector is a triple sign flip. Rather than scatter those flips through the
 * integrator, they live in {@link #toBodyAxes()} alone — one method, one test.
 *
 * <p>Components are held free of {@code −0.0}, for the reason given on {@link SignedZero}.
 */
public record BodyRates(double roll, double pitch, double yaw) {

    public static final BodyRates ZERO = new BodyRates(0, 0, 0);

    /**
     * Not a no-op — see {@link SignedZero}. This record is the third in the family that needs it,
     * because it does its own sign-flipping arithmetic: {@link #toBodyAxes()} negates all three
     * components, and {@code scale} by a negative is how {@code QuadIntegrator} applies angular
     * drag. Without this, {@code ZERO.scale(-1)} does not {@code equals} {@code ZERO}.
     */
    public BodyRates {
        roll = SignedZero.canonical(roll);
        pitch = SignedZero.canonical(pitch);
        yaw = SignedZero.canonical(yaw);
    }

    /**
     * This angular velocity as a vector on the body axes, ready for
     * {@link com.maartenpeels.fpv.math.Quat#integrate}.
     *
     * <p>Roll is a rotation about {@code −Z} (forward), pitch about {@code −X} (right), yaw about
     * {@code −Y} (up) — every one negative, because each pilot-positive direction is the opposite
     * of the right-hand rule about the corresponding positive axis.
     */
    public Vec3 toBodyAxes() {
        return new Vec3(-this.pitch, -this.yaw, -this.roll);
    }

    public BodyRates plus(BodyRates other) {
        return new BodyRates(
                this.roll + other.roll, this.pitch + other.pitch, this.yaw + other.yaw);
    }

    public BodyRates scale(double factor) {
        return new BodyRates(this.roll * factor, this.pitch * factor, this.yaw * factor);
    }

    public boolean isFinite() {
        return Double.isFinite(this.roll) && Double.isFinite(this.pitch) && Double.isFinite(this.yaw);
    }
}
