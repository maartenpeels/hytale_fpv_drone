package com.maartenpeels.fpv.math;

/**
 * An axis-aligned bounding box, stored as its two opposite corners.
 *
 * <p>Stored as {@code min}/{@code max} because that is the shape of the things it describes on the
 * plugin side — a block occupies {@code [x, x+1)} on every axis — and constructed from a centre and
 * half-extents via {@link #centredAt} because that is the shape of a drone. Both readings are always
 * available; neither is privileged beyond which one the record carries.
 *
 * <p><b>Degenerate boxes are allowed.</b> {@code min} equal to {@code max} on an axis describes a
 * flat plane or a line or a point, all of which are reasonable things to ask a question about — a
 * gate plane most obviously. Only an <em>inverted</em> box, with {@code min} greater than
 * {@code max}, is rejected, because that is not a shape, it is a transposed argument.
 *
 * <p>{@code double} throughout, for the reasons on {@link Vec3}.
 */
public record Aabb(Vec3 min, Vec3 max) {

    public Aabb {
        requireFinite(min, "min");
        requireFinite(max, "max");
        if (min.x() > max.x() || min.y() > max.y() || min.z() > max.z()) {
            throw new IllegalArgumentException("min must not exceed max on any axis: " + min + ".." + max);
        }
    }

    /**
     * The box of size {@code 2 * halfExtents} centred on {@code centre} — a drone, or a gate.
     *
     * <p>Zero half-extents give a point, which is a legitimate query: a point sweep is a ray cast.
     */
    public static Aabb centredAt(Vec3 centre, Vec3 halfExtents) {
        requireFinite(centre, "centre");
        requireNonNegative(halfExtents, "halfExtents");
        return new Aabb(centre.minus(halfExtents), centre.plus(halfExtents));
    }

    /** The cube of edge length {@code size} whose lowest corner is {@code min} — a block. */
    public static Aabb cubeAt(Vec3 min, double size) {
        if (!Double.isFinite(size) || size < 0.0) {
            throw new IllegalArgumentException("size must be finite and non-negative but was " + size);
        }
        return new Aabb(min, min.plus(new Vec3(size, size, size)));
    }

    public Vec3 centre() {
        return this.min.plus(this.max).scale(0.5);
    }

    public Vec3 halfExtents() {
        return this.max.minus(this.min).scale(0.5);
    }

    public Vec3 size() {
        return this.max.minus(this.min);
    }

    /**
     * This box grown outward by {@code halfExtents} on every axis.
     *
     * <p>This is the Minkowski sum with a box of those half-extents, and it is what turns
     * box-versus-box into point-versus-box: a box of half-extents {@code h} centred at {@code p}
     * overlaps this box exactly when {@code p} lies inside {@code this.expandedBy(h)}. See
     * {@link com.maartenpeels.fpv.collision.SweptAabb}.
     */
    public Aabb expandedBy(Vec3 halfExtents) {
        requireNonNegative(halfExtents, "halfExtents");
        return new Aabb(this.min.minus(halfExtents), this.max.plus(halfExtents));
    }

    public Aabb translatedBy(Vec3 offset) {
        requireFinite(offset, "offset");
        return new Aabb(this.min.plus(offset), this.max.plus(offset));
    }

    /** Whether {@code point} lies in this box, counting its surface — the box is closed. */
    public boolean contains(Vec3 point) {
        requireFinite(point, "point");
        return point.x() >= this.min.x() && point.x() <= this.max.x()
                && point.y() >= this.min.y() && point.y() <= this.max.y()
                && point.z() >= this.min.z() && point.z() <= this.max.z();
    }

    /**
     * Whether this box and {@code other} share a region of non-zero volume.
     *
     * <p>Strict: boxes that merely touch, sharing a face, edge or corner exactly, do <em>not</em>
     * overlap. That matches {@link com.maartenpeels.fpv.collision.SweptAabb}, which for the same
     * reason treats exact tangency as a miss — a drone resting exactly on the floor is touching it,
     * not inside it. A degenerate box never overlaps anything, for the same reason. Note that this
     * makes {@link #contains} the more generous of the two: a point on the surface is contained, but
     * a zero-volume overlap is not an overlap.
     *
     * <p>Written as {@code max(mins) < min(maxs)} rather than the more familiar
     * {@code a.min < b.max && b.min < a.max}: those two agree only while both boxes are
     * non-degenerate, and the familiar form calls a flat plane inside a block an overlap.
     */
    public boolean overlaps(Aabb other) {
        return Math.max(this.min.x(), other.min.x()) < Math.min(this.max.x(), other.max.x())
                && Math.max(this.min.y(), other.min.y()) < Math.min(this.max.y(), other.max.y())
                && Math.max(this.min.z(), other.min.z()) < Math.min(this.max.z(), other.max.z());
    }

    private static void requireFinite(Vec3 value, String name) {
        if (value == null || !value.isFinite()) {
            throw new IllegalArgumentException(name + " must be finite but was " + value);
        }
    }

    private static void requireNonNegative(Vec3 value, String name) {
        requireFinite(value, name);
        if (value.x() < 0.0 || value.y() < 0.0 || value.z() < 0.0) {
            throw new IllegalArgumentException(
                    name + " must be non-negative on every axis but was " + value);
        }
    }
}
