package com.maartenpeels.fpv.control;

/**
 * One packet's worth of raw pilot input, in world convention, before any interpretation.
 *
 * <p>This is the only type in {@code :fpv-core} that deliberately accepts garbage. Every other value
 * here validates in its constructor, because an out-of-range value means a bug; this one carries
 * numbers a remote client chose, so rejecting them would mean a hostile or merely buggy client could
 * kill a tick. Sanitising is {@link PilotInputMapper}'s job, and the rules it applies are documented
 * on {@link PilotInputMapper#map}.
 *
 * <h2>Conventions</h2>
 *
 * The plugin adapter copies packet fields into this record and converts nothing — not units, not
 * signs. So these are stated in the world convention {@code :fpv-core} shares with Hytale:
 *
 * <ul>
 *   <li>{@code +Y} up, forward is {@code −Z}, the aircraft's right is {@code +X}.
 *   <li>Yaw is a right-handed rotation about {@code +Y}, so <strong>increasing yaw turns the nose
 *       left</strong>.
 *   <li>Pitch is positive nose-<strong>up</strong>.
 *   <li>All angles are in <strong>radians</strong>. Verified: {@code Vector3dUtil.setYawPitch} feeds
 *       {@code TrigMathUtil.sin/cos}, whose parameters are declared {@code radians}, and
 *       {@code Rotation3f.addRotationOnAxis(Axis, int degrees)} multiplies by {@code π/180} before
 *       storing — so the stored representation is radians and degrees are what needs converting.
 * </ul>
 *
 * <p>Note that none of this is {@link ControlInput}'s convention, which is a transmitter's: positive
 * pitch nose-down, positive yaw nose-right. The mapper is where the two meet.
 *
 * <h2>{@code NaN} means "absent", not "broken"</h2>
 *
 * Every field of Hytale's {@code ClientMovement} is nullable, and the client omits
 * {@code lookOrientation} when the look has not changed — so a missing look is the common case, not
 * an error. {@code NaN} in {@link #lookYaw} or {@link #lookPitch} means this packet carried no look
 * angle, and the mapper answers zero deflection while keeping its memory of the last real angle.
 * {@code NaN} in {@link #wishFrameYaw} means the frame the wish vector was rotated into is unknown
 * from this packet alone.
 *
 * @param wishX world-space {@code x} of the client's desired-movement vector
 * @param wishZ world-space {@code z} of the same; {@code y} is deliberately not carried — see
 *     {@code docs/plans/17.md}
 * @param wishFrameYaw the yaw the wish vector was rotated into by the client, radians, or
 *     {@code NaN} if unknown. Hytale's default {@code MovementForceRotationType.AttachedToHead}
 *     makes this the head yaw; a camera pinned with {@code Custom} and a zero rotation makes it a
 *     constant {@code 0}.
 * @param lookYaw absolute look yaw, radians, or {@code NaN} if this packet carried none
 * @param lookPitch absolute look pitch, radians, or {@code NaN} if this packet carried none
 */
public record PilotInputSample(
        double wishX, double wishZ, double wishFrameYaw, double lookYaw, double lookPitch) {

    /**
     * A packet that carried nothing: sticks centred, look unchanged, frame unknown. What the adapter
     * produces from a {@code ClientMovement} whose movement fields were all null.
     */
    public static final PilotInputSample EMPTY =
            new PilotInputSample(0.0, 0.0, Double.NaN, Double.NaN, Double.NaN);

    /**
     * A sample whose wish vector is expressed in the pilot's own look frame — Hytale's default,
     * where the client rotates the stick vector by the head rotation before sending it.
     */
    public static PilotInputSample lookRelative(
            double wishX, double wishZ, double lookYaw, double lookPitch) {
        return new PilotInputSample(wishX, wishZ, lookYaw, lookYaw, lookPitch);
    }

    /** True when this packet carried a usable look orientation. */
    public boolean hasLook() {
        return Double.isFinite(this.lookYaw) && Double.isFinite(this.lookPitch);
    }

    /**
     * The same sample with its look angles erased — the wish axes kept, the look reported as absent.
     *
     * <p>This is what a tick that received <em>no packet at all</em> feeds the mapper when it wants to
     * hold the left stick where the pilot left it. A physical throttle and yaw stick stay put when a
     * hand comes off them; pitch and roll ride a look delta, and no new look means no deflection.
     *
     * <p><strong>Why not just re-feed the whole sample.</strong> Doing that looks equivalent — the
     * look angles now equal the tracked ones, so the delta is zero — but {@link PilotInputMapper#map}
     * answers a {@link LookTrack} sampled <em>now</em>, resetting {@link LookTrack#secondsSinceSample}
     * to zero. Five quiet ticks followed by one real sample would then divide a five-tick delta by one
     * tick's {@code dt}, over-reporting the look rate five-fold. That is the exact bug #17's
     * self-review caught, reintroduced through the retry path. Erasing the look angles takes
     * {@code map}'s {@link LookTrack#aged} branch instead, which accumulates the interval the next
     * delta will really have spanned.
     *
     * <p>{@link #wishFrameYaw} is kept. It was resolved from a real look yaw when the packet arrived,
     * {@code map} prefers it over the look angles anyway, and dropping it would leave the wish vector
     * uninterpretable — a world-space vector says nothing without the frame it was rotated into.
     */
    public PilotInputSample withoutLook() {
        if (!Double.isFinite(this.lookYaw) && !Double.isFinite(this.lookPitch)) {
            return this;
        }
        return new PilotInputSample(
                this.wishX, this.wishZ, this.wishFrameYaw, Double.NaN, Double.NaN);
    }
}
