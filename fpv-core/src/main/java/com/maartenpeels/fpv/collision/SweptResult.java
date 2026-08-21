package com.maartenpeels.fpv.collision;

import com.maartenpeels.fpv.math.Vec3;

/**
 * The answer {@link SweptAabb#sweep} gives: what a moving box did to a stationary box over one
 * segment of motion.
 *
 * <p>Three cases and not two, because {@link AlreadyOverlapping} genuinely has nothing to say about
 * a face or a time of impact — the mover did not cross anything, it started inside. Reporting a
 * normal there would mean inventing one, and reporting it as a {@link Miss} would hide it. Both
 * callers care about the difference: for terrain collision it means the drone spawned in a block or
 * the previous tick failed to resolve, and for gate crossing it means the pass was already under way
 * when the segment started.
 */
public sealed interface SweptResult {

    /** The one {@link Miss}; the type carries no state, so there is no reason for a second. */
    Miss MISS = new Miss();

    /** The one {@link AlreadyOverlapping}. */
    AlreadyOverlapping ALREADY_OVERLAPPING = new AlreadyOverlapping();

    /** The boxes never shared a region of non-zero volume at any point of the segment. */
    record Miss() implements SweptResult {}

    /**
     * The boxes already overlapped when the segment began.
     *
     * <p>No time and no normal, deliberately: there is no face the mover entered through. A caller
     * that needs a direction to push out along should compute it from the two boxes, which is a
     * static question about penetration depth and not a swept one.
     */
    record AlreadyOverlapping() implements SweptResult {}

    /**
     * The mover entered the target during the segment, at {@code entryTime}, through the face whose
     * outward normal is {@code normal}.
     *
     * @param entryTime fraction of the segment at which contact begins, in {@code [0, 1)}. The
     *     impact position is {@code from.plus(displacement.scale(entryTime))}. It is a solved time
     *     rather than a sampled one, which is what makes the routine tunnel-proof.
     * @param exitTime fraction of the segment at which the mover would leave the target,
     *     <b>deliberately not clamped to 1</b>. Greater than {@code 1} means the mover was still
     *     inside the target at the end of the segment — read as "ended embedded" by terrain
     *     collision and as "the pass has not completed yet" by gate crossing. Always greater than
     *     {@code entryTime}, and permitted to be positive infinity: at a displacement small enough
     *     for the quotient to overflow, the mover really does not leave within any finite multiple of
     *     the segment, and infinity is the answer rather than a failure.
     * @param normal the <b>outward</b> unit normal of the target face that was entered, so it points
     *     back toward the mover and opposes the motion along its own axis. That is the sign
     *     collision response wants, and the sign {@link #enteredAlong} reads for direction.
     *     {@link SweptAabb} always produces an axis-aligned normal; the invariant enforced here is
     *     only that it is unit length, so a caller working in a rotated frame can rotate one back
     *     into world space and rebuild.
     */
    record Contact(double entryTime, double exitTime, Vec3 normal) implements SweptResult {

        /** Slack on the unit-length check: rotating a normal by a quaternion costs a few ULP. */
        private static final double NORMAL_TOLERANCE = 1.0e-9;

        public Contact {
            // A displacement that is negative on the entering axis divides an exact zero into −0.0,
            // which compares unequal to 0.0 under IEEE bit equality. No caller should have to know.
            entryTime = entryTime == 0.0 ? 0.0 : entryTime;

            if (!Double.isFinite(entryTime) || entryTime < 0.0 || entryTime >= 1.0) {
                throw new IllegalArgumentException(
                        "entryTime must be a fraction of the segment in [0, 1) but was " + entryTime);
            }
            if (Double.isNaN(exitTime) || exitTime <= entryTime) {
                throw new IllegalArgumentException(
                        "exitTime must be after entryTime " + entryTime + " but was " + exitTime);
            }
            if (normal == null || !normal.isFinite()
                    || Math.abs(normal.length() - 1.0) > NORMAL_TOLERANCE) {
                throw new IllegalArgumentException("normal must be a unit vector but was " + normal);
            }
        }

        /**
         * Whether the mover entered while travelling broadly along {@code direction}.
         *
         * <p>This is the gate direction test. A gate's forward vector is the direction a pilot is
         * required to fly through it; flying it correctly means approaching from the far side, which
         * means entering through the face whose outward normal opposes that vector. So a correct pass
         * is {@code normal · direction < 0}, and this method is that.
         *
         * <p>A wrong-way pass returns {@code false} because the normal is the other face's. A purely
         * sideways clip also returns {@code false}, because the dot product is zero and the mover
         * travelled neither way along {@code direction}.
         *
         * @param direction need not be unit length; only the sign of the projection is used
         */
        public boolean enteredAlong(Vec3 direction) {
            if (direction == null || !direction.isFinite()) {
                throw new IllegalArgumentException("direction must be finite but was " + direction);
            }
            return this.normal.dot(direction) < 0.0;
        }

        /**
         * Whether the mover was clear of the target again by the end of the segment.
         *
         * <p>A completed gate pass, or a terrain hit that did not leave the drone embedded. The
         * negation is the interesting one for terrain: {@code false} means the segment ended with the
         * boxes still overlapping.
         */
        public boolean passedFullyThrough() {
            return this.exitTime <= 1.0;
        }

        /** Where the mover's centre was at the moment of contact. */
        public Vec3 positionAt(Vec3 from, Vec3 displacement) {
            return from.plus(displacement.scale(this.entryTime));
        }
    }
}
