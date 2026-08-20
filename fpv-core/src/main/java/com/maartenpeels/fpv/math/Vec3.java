package com.maartenpeels.fpv.math;

/**
 * An immutable three-component vector in {@code double} precision.
 *
 * <p>{@code double} rather than {@code float} for two reasons: Hytale's own
 * {@code protocol.Position} is {@code double}, so nothing is lost at the packet boundary; and the
 * integrator runs at ~240 steps per second, where {@code float} position accumulates drift a
 * pilot can see.
 *
 * <p>The flight model's frame convention — right-handed, {@code +Y} up, {@code −Z} forward,
 * {@code +X} right — is documented on {@link com.maartenpeels.fpv.flight.DroneState}. This type
 * is frame-agnostic; it is used for both world-frame and body-frame quantities.
 */
public record Vec3(double x, double y, double z) {

    public static final Vec3 ZERO = new Vec3(0, 0, 0);

    /** World up, and the axis a level drone's thrust points along. */
    public static final Vec3 UP = new Vec3(0, 1, 0);

    public Vec3 plus(Vec3 other) {
        return new Vec3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public Vec3 minus(Vec3 other) {
        return new Vec3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    public Vec3 scale(double factor) {
        return new Vec3(this.x * factor, this.y * factor, this.z * factor);
    }

    public Vec3 negated() {
        return new Vec3(-this.x, -this.y, -this.z);
    }

    public double dot(Vec3 other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    public Vec3 cross(Vec3 other) {
        return new Vec3(
                this.y * other.z - this.z * other.y,
                this.z * other.x - this.x * other.z,
                this.x * other.y - this.y * other.x);
    }

    public double lengthSquared() {
        return this.dot(this);
    }

    public double length() {
        return Math.sqrt(this.lengthSquared());
    }

    /** Unit vector in the same direction, or {@link #ZERO} for a zero-length vector. */
    public Vec3 normalised() {
        double length = this.length();
        return length == 0.0 ? ZERO : this.scale(1.0 / length);
    }

    public boolean isFinite() {
        return Double.isFinite(this.x) && Double.isFinite(this.y) && Double.isFinite(this.z);
    }
}
