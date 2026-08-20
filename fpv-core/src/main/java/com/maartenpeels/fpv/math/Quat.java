package com.maartenpeels.fpv.math;

/**
 * A rotation, stored as a quaternion.
 *
 * <p>A quadcopter flies inverted, loops and rolls through vertical, so the attitude
 * representation has to survive any orientation. Euler angles gimbal-lock at ±90° of pitch, which
 * is a normal part of a flip — hence a quaternion.
 *
 * <p>Instances are expected to be unit-length. The constructor does not normalise, because
 * normalising on every construction would hide accumulating error rather than let
 * {@link #integrate} deal with it explicitly.
 *
 * <p>Deliberately no Euler-angle readout. Hytale's {@code Direction{yaw, pitch, roll}} has its own
 * sign conventions (verified: {@code Direction.pitch} is positive nose-<em>up</em>, the opposite of
 * {@link com.maartenpeels.fpv.control.ControlInput}), and that mapping belongs to the plugin-side
 * camera adapter where the packet can actually be tested against.
 */
public record Quat(double w, double x, double y, double z) {

    public static final Quat IDENTITY = new Quat(1, 0, 0, 0);

    /** A rotation of {@code radians} about {@code axis}, by the right-hand rule. */
    public static Quat fromAxisAngle(Vec3 axis, double radians) {
        Vec3 unit = axis.normalised();
        if (unit.equals(Vec3.ZERO)) {
            return IDENTITY;
        }
        double half = radians * 0.5;
        double sin = Math.sin(half);
        return new Quat(Math.cos(half), unit.x() * sin, unit.y() * sin, unit.z() * sin);
    }

    /** Hamilton product: the rotation {@code other} followed by {@code this}. */
    public Quat times(Quat other) {
        return new Quat(
                this.w * other.w - this.x * other.x - this.y * other.y - this.z * other.z,
                this.w * other.x + this.x * other.w + this.y * other.z - this.z * other.y,
                this.w * other.y - this.x * other.z + this.y * other.w + this.z * other.x,
                this.w * other.z + this.x * other.y - this.y * other.x + this.z * other.w);
    }

    /** The inverse rotation, valid for unit quaternions. */
    public Quat conjugate() {
        return new Quat(this.w, -this.x, -this.y, -this.z);
    }

    public double norm() {
        return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
    }

    /** Unit-length equivalent, falling back to {@link #IDENTITY} if this has collapsed to zero. */
    public Quat normalised() {
        double norm = this.norm();
        if (norm == 0.0 || !Double.isFinite(norm)) {
            return IDENTITY;
        }
        double inverse = 1.0 / norm;
        return new Quat(this.w * inverse, this.x * inverse, this.y * inverse, this.z * inverse);
    }

    /** Rotates a body-frame vector into the world frame. */
    public Vec3 rotate(Vec3 v) {
        // v + 2q_v × (q_v × v + w·v), the standard reduced form -- fewer operations and less
        // rounding than building a matrix.
        Vec3 axis = new Vec3(this.x, this.y, this.z);
        Vec3 t = axis.cross(v).plus(v.scale(this.w)).scale(2.0);
        return v.plus(axis.cross(t));
    }

    /** Rotates a world-frame vector into the body frame. */
    public Vec3 inverseRotate(Vec3 v) {
        return this.conjugate().rotate(v);
    }

    /**
     * Advances this orientation by {@code dt} seconds of rotation at {@code bodyRates} (rad/s,
     * expressed on the body axes).
     *
     * <p>First-order in {@code dt}, with an explicit renormalisation: the linear update leaves the
     * quaternion slightly off the unit sphere, and letting that accumulate would slowly scale the
     * drone's apparent rotation rate.
     */
    public Quat integrate(Vec3 bodyRates, double dt) {
        Quat rate = new Quat(0, bodyRates.x(), bodyRates.y(), bodyRates.z());
        Quat derivative = this.times(rate);
        double half = 0.5 * dt;
        return new Quat(
                        this.w + derivative.w * half,
                        this.x + derivative.x * half,
                        this.y + derivative.y * half,
                        this.z + derivative.z * half)
                .normalised();
    }
}
