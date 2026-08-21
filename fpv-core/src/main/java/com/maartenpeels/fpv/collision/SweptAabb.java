package com.maartenpeels.fpv.collision;

import com.maartenpeels.fpv.math.Aabb;
import com.maartenpeels.fpv.math.Vec3;

/**
 * The swept box-versus-box test: does a box, moved along one straight segment, meet a stationary box
 * — and if so, when, and through which face.
 *
 * <p>Two callers by design. Terrain collision asks whether the drone hit a block, and needs the time
 * of impact and a normal to resolve it. Gate crossing asks whether the drone passed a gate the right
 * way round, and reads the direction off the same normal via
 * {@link SweptResult.Contact#enteredAlong}. Building it once is why this lives in the pure module
 * with unit tests rather than twice behind a server.
 *
 * <h2>Why swept rather than sampled</h2>
 *
 * A drone at racing speed covers many block widths in one simulation step. Testing whether the boxes
 * overlap at the start of the step and at the end of it misses every wall thinner than the step —
 * the drone tunnels straight through. This routine <em>solves</em> for the time of impact instead of
 * sampling for it, so a 20-unit step through a 1-unit wall is found exactly the same way a
 * 0.02-unit step is.
 *
 * <h2>How</h2>
 *
 * Growing the target by the mover's half-extents and shrinking the mover to its centre point is an
 * exact restatement of the problem, not an approximation: the two boxes overlap precisely when the
 * mover's centre lies inside the grown box. That reduces the sweep to a segment against one box, and
 * a segment against an axis-aligned box is three independent one-dimensional intervals — the mover
 * is inside the box over exactly the times it is inside all three slabs at once. So the answer is
 * {@code max} of the three entry times against {@code min} of the three exit times.
 *
 * <h2>Both boxes are axis-aligned; rotation is the caller's job</h2>
 *
 * A gate has an orientation. It is still handled here, exactly, by transforming the segment into the
 * gate's own frame — where the gate <em>is</em> axis-aligned — and rotating the returned normal back
 * out. A rigid rotation maps the problem onto this one without loss.
 *
 * <p>The drone, on the other hand, is deliberately treated as a non-rotating world-aligned box. Its
 * true silhouette changes with attitude, and sweeping a rotating box has no closed-form time of
 * impact; a slightly generous box is the conventional trade and it is ample for deciding whether a
 * quad hit a wall.
 *
 * <h2>The rule that settles every edge case</h2>
 *
 * <b>An intersection is a non-empty open interval of genuine, positive-volume overlap inside
 * {@code (0, 1)}.</b> One rule, no epsilon, no skin width — which matters, because an epsilon here
 * would be a feel-affecting constant buried in geometry. Everything awkward follows from it:
 *
 * <ul>
 *   <li>Tangency <b>counts</b> on an axis the mover is crossing — that time <em>is</em> the time of
 *       impact, so a head-on hit could not be found otherwise.
 *   <li>Tangency <b>does not count</b> on an axis the mover moves parallel to. A drone resting
 *       exactly on the floor and sliding along it is touching the floor, not inside it, and must not
 *       report a hit every tick.
 *   <li>An exact corner graze, where the entry and exit times coincide, is a {@link SweptResult#MISS}
 *       — zero duration of contact is not a collision.
 *   <li>Arriving exactly tangent at the end of the segment is a miss this step and a contact at
 *       {@code entryTime == 0} on the next one, so nothing is skipped.
 *   <li>Zero displacement degenerates into a plain overlap test, with no special case.
 * </ul>
 *
 * <p>This reports <em>contact</em>, not <em>crash</em>. A drone descending onto the floor at any
 * speed at all is genuinely moving into it and does get a {@link SweptResult.Contact}, correctly —
 * that is how landing works. Whether a contact is a crash is a question about impact speed, and it
 * belongs to the caller.
 */
public final class SweptAabb {

    private SweptAabb() {}

    /**
     * Sweeps {@code mover} through {@code displacement} against {@code target}.
     *
     * @param mover the moving box at the <em>start</em> of the segment
     * @param displacement how far it travels over the segment; {@code to.minus(from)}
     * @param target the stationary box, in the same frame
     * @throws IllegalArgumentException if {@code displacement} is not finite. Left to fail loudly
     *     rather than degrade, because every comparison against {@code NaN} is false, so a
     *     {@code NaN} displacement would sail through the interval logic and be reported as
     *     {@link SweptResult#ALREADY_OVERLAPPING}.
     */
    public static SweptResult sweep(Aabb mover, Vec3 displacement, Aabb target) {
        if (mover == null || target == null) {
            throw new IllegalArgumentException("mover and target must not be null");
        }
        if (displacement == null || !displacement.isFinite()) {
            throw new IllegalArgumentException("displacement must be finite but was " + displacement);
        }

        Aabb expanded = target.expandedBy(mover.halfExtents());
        Vec3 origin = mover.centre();

        double entry = Double.NEGATIVE_INFINITY;
        double exit = Double.POSITIVE_INFINITY;
        Vec3 entryNormal = null;

        for (int axis = 0; axis < 3; axis++) {
            double travel = component(displacement, axis);
            double toNear = component(expanded.min(), axis) - component(origin, axis);
            double toFar = component(expanded.max(), axis) - component(origin, axis);

            if (travel == 0.0) {
                // Parallel to this slab, so the overlap on this axis never changes. Strict, because
                // sitting exactly on a bound is zero-volume contact.
                if (toNear >= 0.0 || toFar <= 0.0) {
                    return SweptResult.MISS;
                }
                continue;
            }

            double axisEntry;
            double axisExit;
            double outwardSign;
            if (travel > 0.0) {
                axisEntry = toNear / travel;
                axisExit = toFar / travel;
                outwardSign = -1.0;
            } else {
                axisEntry = toFar / travel;
                axisExit = toNear / travel;
                outwardSign = 1.0;
            }

            if (axisEntry > entry) {
                // Strict, so on an exact corner approach the earlier axis keeps the normal: X, then
                // Y, then Z. Arbitrary but fixed, and the same order the server's own evaluator uses.
                entry = axisEntry;
                entryNormal = outwardNormal(axis, outwardSign);
            }
            if (axisExit < exit) {
                exit = axisExit;
            }
            if (entry >= exit) {
                return SweptResult.MISS;
            }
        }

        if (entry >= 1.0 || exit <= 0.0) {
            return SweptResult.MISS;
        }
        if (entry < 0.0) {
            return SweptResult.ALREADY_OVERLAPPING;
        }
        return new SweptResult.Contact(entry, exit, entryNormal);
    }

    private static double component(Vec3 vector, int axis) {
        return switch (axis) {
            case 0 -> vector.x();
            case 1 -> vector.y();
            case 2 -> vector.z();
            default -> throw new IllegalStateException("no axis " + axis);
        };
    }

    private static Vec3 outwardNormal(int axis, double sign) {
        return switch (axis) {
            case 0 -> new Vec3(sign, 0, 0);
            case 1 -> new Vec3(0, sign, 0);
            case 2 -> new Vec3(0, 0, sign);
            default -> throw new IllegalStateException("no axis " + axis);
        };
    }
}
