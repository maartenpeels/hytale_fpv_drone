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
 * A gate has an orientation. It is handled by transforming the segment into the gate's own frame —
 * where the gate <em>is</em> axis-aligned — and reading the result there. That reduction is exact
 * <b>for the gate box and for the segment</b>: a rigid rotation maps both onto this problem without
 * loss. A caller doing that does not even need to rotate the normal back out, because the gate's
 * forward vector is a constant axis in its own frame, so {@link SweptResult.Contact#enteredAlong}
 * works directly on the local-frame normal.
 *
 * <p>It is <b>not</b> exact for the mover. The drone is deliberately treated as a non-rotating
 * world-aligned box, so in a rotated frame it is handed over with its world half-extents rather than
 * its true ones — the honest gate-frame half-extent on axis {@code i} is
 * {@code Σⱼ |Rᵢⱼ|·hⱼ}. Nor does that error point consistently one way: it inflates the box on some
 * axes and shrinks it on others, so a yawed gate is very slightly easier to clip the edge of and
 * very slightly harder to catch an edge on. For a quad a few tens of centimetres across against a
 * gate measured in metres this is a rounding error on a rounding error, but it is an approximation
 * and not an identity, and #6 should not be told otherwise.
 *
 * <h2>The rule that settles every edge case</h2>
 *
 * <b>An intersection is a non-empty open interval, inside {@code (0, 1)}, over which the mover's
 * centre lies strictly within the expanded target.</b> One rule, no epsilon, no skin width — which
 * matters, because an epsilon here would be a feel-affecting constant buried in geometry.
 *
 * <p>Positive <em>duration</em>, note, not positive <em>volume</em>. The distinction only shows up
 * for degenerate boxes, and there it matters: a zero-extent point mover crossing a block, or a real
 * drone crossing a zero-thickness gate plane, shares no volume at any instant yet is a genuine
 * crossing, and this routine reports it as one. That is why {@link Aabb#overlaps} — which does
 * measure volume — deliberately disagrees here, and why it must not be used as a pre-check beside a
 * sweep. The one combination that falls out as always-miss is a point mover against a degenerate
 * target, since the expanded target is then still flat and a point cannot be strictly inside it.
 *
 * <p>Everything awkward follows from the rule:
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
     *     {@link SweptResult#ALREADY_OVERLAPPING}. Also thrown, from {@link Aabb} itself, if either
     *     box is so large that expanding the target by the mover's half-extents overflows a
     *     {@code double} — which needs coordinates around {@code 1e308} and so cannot arise from
     *     world geometry, but is worth knowing is an exception rather than an infinity.
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
