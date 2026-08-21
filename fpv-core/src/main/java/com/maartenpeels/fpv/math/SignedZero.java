package com.maartenpeels.fpv.math;

/**
 * Collapses IEEE 754's negative zero onto positive zero, so that a value record holding
 * {@code double}s can be compared for equality without a sign bit deciding the answer.
 *
 * <h2>The problem this exists for</h2>
 *
 * {@code −0.0} and {@code 0.0} are numerically identical and behave identically in arithmetic:
 * {@code -0.0 == 0.0} is {@code true}. But <b>a record's generated {@code equals} compares
 * {@code double} components with {@code Double.compare}, not {@code ==}</b> (JLS 8.10.3), and
 * {@code Double.compare(-0.0, 0.0)} is {@code −1}. So the two are unequal there, and
 * {@code Double.hashCode} puts them in different buckets.
 *
 * <p>Negating an exact zero produces {@code −0.0}, and so does scaling one by a negative, or
 * multiplying anything by zero. That makes <b>axis-aligned unit vectors the worst case</b> — two of
 * their three components are exactly zero — which is unfortunate, because axis-aligned normals are
 * what collision and gate crossing compare, switch on, and use as map keys. Without this,
 * {@code new Vec3(1, 0, 0).negated().equals(new Vec3(-1, 0, 0))} is {@code false}, and the symptom
 * is a branch that will not fire while the value looks perfectly correct in a debugger. See #38.
 *
 * <h2>Why {@code value + 0.0} and not a comparison</h2>
 *
 * <p><b>This is not dead code, and it is not a no-op. Do not "simplify" it to {@code return
 * value}.</b> Adding positive zero is the identity function on every input except the one that needs
 * changing, which is exactly the operation wanted:
 *
 * <ul>
 *   <li>{@code −0.0 + 0.0} is {@code +0.0} — the whole point.
 *   <li>{@code x + 0.0} is {@code x} <em>exactly</em> for every non-zero finite {@code x},
 *       subnormals included. The exact sum <em>is</em> {@code x} and {@code x} is representable, so
 *       there is no rounding step in which anything could be lost.
 *   <li>{@code ±∞} and {@code NaN} survive unchanged, so {@code isFinite} checks downstream still
 *       see what they were given. (Record equality already treats every {@code NaN} as equal to
 *       every other, via {@code doubleToLongBits}, so {@code NaN} was never part of the problem.)
 * </ul>
 *
 * <p>The obvious alternative, {@code value == 0.0 ? 0.0 : value}, is equally correct and reads more
 * plainly. It was measured on {@code QuadIntegrator.step} at <b>+47.7% over an unnormalised
 * baseline, against +17.7% for the add</b> — so nearly three times the overhead for the same
 * guarantee. (The likely reason is that a compare-and-select blocks HotSpot from scalar-replacing
 * these short-lived records while a bare add does not, but that is inference from the shape of the
 * result and was not confirmed against JIT output. Nothing here depends on the explanation being
 * right; the measurement stands either way.)
 *
 * <p>Both idioms were also checked to survive C2 rather than being folded away — 200M iterations
 * each under {@code -XX:-TieredCompilation}, bit-checked, zero failures. Full numbers and method are
 * in {@code docs/plans/38.md}; {@code SignedZeroTest} keeps the property pinned.
 *
 * <h2>Where it belongs</h2>
 *
 * In the <b>canonical constructor</b> of the value record, not in individual operations. Doing it in
 * {@code negated()} alone leaves the same trap in {@code scale}, {@code cross}, {@code conjugate}
 * and in every operation not written yet; the constructor is the one place that catches all of them.
 * The cost of that — every construction, at ~240 integration steps per second per pilot — is 0.0016%
 * of a 30 TPS tick budget at the eight-pilot ceiling, which is why it is affordable to do properly.
 *
 * <p>It is applied by {@link Vec3}, {@link Quat} and {@link com.maartenpeels.fpv.flight.BodyRates}.
 * The rule for what qualifies: <b>a value record that does its own sign-flipping arithmetic and has
 * a canonical constant callers compare against.</b> A record that merely carries computed signals
 * does not need it, because nothing compares one against a literal — which is why
 * {@code TorqueDemand}, {@code PidState}, {@code MotorThrusts} and {@code MotorOutputs} are left
 * alone. Every record in {@code :fpv-core} was checked against that rule for #38; no other one
 * matches.
 *
 * <h2>Two places this deliberately does not reach</h2>
 *
 * <ul>
 *   <li>{@code Contact}'s {@code entryTime} in {@link com.maartenpeels.fpv.collision.SweptResult} is
 *       a bare {@code double} component, not a {@code Vec3}, so no constructor upstream can
 *       canonicalise it. It calls {@link #canonical} directly.
 *   <li>{@code ControlInput} is {@code float}, and its zeros arrive from a wire format rather than
 *       from its own arithmetic — it has none. The {@code −0.0f} is scrubbed by its <em>producer</em>,
 *       {@code PilotInputMapper.stick}, which is where the rotation and negation that create it live.
 *       Same hazard, handled at the point of creation because the type itself never creates one.
 * </ul>
 */
public final class SignedZero {

    private SignedZero() {}

    /** {@code value}, with {@code −0.0} mapped onto {@code 0.0} and everything else untouched. */
    public static double canonical(double value) {
        return value + 0.0;
    }
}
