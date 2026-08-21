package com.maartenpeels.fpv.control;

/**
 * The packet boundary's arithmetic: a raw {@link PilotInputSample} becomes a {@link ControlInput}.
 *
 * <p>Decision 1 says nothing downstream of {@code ControlInput} sees a packet, and decision 10 says
 * the interesting math lives in {@code :fpv-core}. Both point here: the plugin unpacks
 * {@code ClientMovement}'s fields and calls this, and every remap, sign flip, rotation and clamp is
 * under unit test with no server in sight.
 *
 * <h2>Which stick is which</h2>
 *
 * Hytale gives exactly two continuous channels, and they happen to be a Mode 2 transmitter:
 *
 * <ul>
 *   <li>{@code wishMovement} is the <b>left stick</b> — WASD, or a gamepad's left stick. Its forward
 *       axis carries <b>throttle</b>, its lateral axis carries <b>yaw</b>.
 *   <li>{@code lookOrientation} is the <b>right stick</b> — the mouse, or a gamepad's right stick.
 *       Its pitch channel carries <b>pitch</b>, its yaw channel carries <b>roll</b>.
 * </ul>
 *
 * <p>Left gives throttle and yaw, right gives pitch and roll: that is a Mode 2 layout, axis for
 * axis, which is why the assignment is worth spelling out rather than just doing.
 *
 * <h2>Throttle is remapped bipolar, not clipped</h2>
 *
 * {@code throttle = (forward + 1) / 2}. A quad's throttle is unidirectional, so it is tempting to
 * write {@code max(0, forward)} — but the stick feeding it is <em>spring-centred</em>, and a
 * spring-centred throttle rests at mid-stick, not at motors-off. Clipping would throw away half the
 * travel and make mid-throttle the one position a pilot cannot hold. The bipolar remap also degrades
 * correctly onto a keyboard, which matters because that is how #24 gets flown first: no key is
 * mid-stick, {@code W} climbs, {@code S} descends.
 *
 * <h2>Pitch and roll come from the look <em>delta</em>, not the look angle</h2>
 *
 * {@code lookOrientation} is an absolute orientation, so this is a real choice; the full argument is
 * in {@code docs/plans/17.md}. The short form:
 *
 * <ul>
 *   <li><b>Roll leaves no alternative.</b> Nothing a pilot can touch produces camera roll, so
 *       {@code Direction.roll} is dead in both input modes and roll has to ride the look-yaw channel.
 *       Look yaw is unbounded and wrapping, so an absolute mapping would mean "how far you are rolled
 *       depends on which way you are facing", which is not a control.
 *   <li><b>A delta <em>is</em> a self-centring stick.</b> Downstream, these are stick positions that
 *       #15's rate curve turns into demanded angular rates — so half stick means "keep rotating", and
 *       the physical stick being modelled springs back to centre. "The pilot stopped moving the
 *       mouse" maps onto "the stick returned to centre" exactly. An absolute angle would be
 *       angle-mode-shaped, and there is no angle mode here.
 *   <li><b>The pilot cannot see their own head.</b> Decision 4 attaches the camera to the drone, so
 *       the player's own look orientation is invisible. An absolute axis with no indicator is a stick
 *       whose position you cannot know, with headroom that depends on invisible history. Under a
 *       delta the feedback is in your hand.
 *   <li><b>It is the one rule that fits both input devices.</b> A mouse produces unbounded relative
 *       motion, for which absolute yaw is meaningless. A gamepad right stick self-centres and the
 *       client turns its deflection into a look <em>rate</em>, so holding it right yields a constant
 *       delta per tick — a constant deflection. Decision 1's bet that only this adapter changes when
 *       gamepads land only pays off if the mapping works for both, and controllers are still
 *       unverified on hardware here.
 * </ul>
 *
 * <p>The cost is that a sustained pitch input is impossible: mouse travel runs out, and at the pitch
 * stop the client stops producing deltas. Accepted, because in rate mode a pitch input <em>is</em>
 * transient — flick to set attitude, centre, the drone holds it. If #24 disagrees, packet 111's
 * {@code MouseMotionEvent.relativeMotion} is the documented plan B.
 *
 * <p>Because it is a delta, it is divided by {@code dt} to become an angular rate before being
 * normalised. That keeps sensitivity independent of tick rate, so decision 3's escape hatch
 * ({@code World.setTps(120..240)}) stays a config change rather than a silent sensitivity multiplier.
 *
 * <h2>Signs</h2>
 *
 * CLAUDE.md warns that Hytale's {@code Direction} disagrees with {@code ControlInput} on two axes out
 * of three. Concretely, with {@code Vector3dUtil}'s {@code FORWARD = (0,0,−1)} and
 * {@code RIGHT = (1,0,0)}, increasing Hytale yaw takes forward toward {@code −X}, which is the
 * aircraft's <em>left</em>; and Hytale pitch is positive nose-<em>up</em>. So:
 *
 * <ul>
 *   <li>{@code roll = −Δlook.yaw} — mouse right decreases yaw, and transmitter-positive roll banks
 *       right.
 *   <li>{@code pitch = −Δlook.pitch} — {@code ControlInput.pitch} is positive nose-<em>down</em>.
 *   <li>{@code yaw = +lateral}, <b>not</b> negated. The negation rule is about Hytale
 *       <em>angles</em>; the lateral axis is a spatial direction, and {@code +X} is the aircraft's
 *       right. Strafe-right is yaw-right is positive {@code ControlInput.yaw}. Negating here would be
 *       a correct rule applied to the wrong thing.
 * </ul>
 *
 * <p>Immutable and stateless; the memory is the {@link LookTrack} passed in and returned. One
 * instance serves every pilot flying the same sensitivities.
 */
public final class PilotInputMapper {

    private static final double TWO_PI = 2.0 * Math.PI;

    private final PilotInputMapping mapping;

    public PilotInputMapper(PilotInputMapping mapping) {
        if (mapping == null) {
            throw new IllegalArgumentException("mapping must not be null");
        }
        this.mapping = mapping;
    }

    public PilotInputMapping mapping() {
        return this.mapping;
    }

    /**
     * Turns one packet's raw numbers into stick positions, and answers the look memory to carry into
     * the next packet.
     *
     * <p>The sample is untrusted, so nothing in here throws on its account. What it does instead:
     *
     * <ul>
     *   <li>A non-finite wish component counts as zero <em>before</em> the bipolar remap, so throttle
     *       rests at mid-stick. Letting {@code NaN} reach {@link ControlInput#clamped} would collapse
     *       throttle to closed — cutting the motors is the worst available response to one bad packet.
     *   <li>An out-of-range wish component saturates its axis via {@link ControlInput#clamped}.
     *   <li>An absent or non-finite look angle gives zero pitch/roll deflection and <em>keeps</em> the
     *       existing track, {@link LookTrack#aged} by {@code dt}. Absent genuinely means "the look did
     *       not change" — {@code lookOrientation} is nullable and the client omits it when nothing
     *       moved — so the next real sample measures from the last known angle, over the interval it
     *       really spanned rather than one tick.
     *   <li>An absent or non-finite wish frame yaw falls back to this sample's look yaw, then to the
     *       tracked yaw. If none of the three is known the wish vector is <em>uninterpretable</em> —
     *       a world-space vector says nothing without the frame it was rotated into — so both wish
     *       axes read as centred. Guessing a heading of zero instead would give a pilot facing east
     *       full yaw-right and no throttle for holding {@code W}.
     *   <li>The first sample after a reset gives centred pitch and roll, because there is nothing to
     *       difference against yet. Arming a drone must not fire a flick.
     *   <li>A yaw delta past ±π wraps into {@code (−π, π]} — the pilot took the short way round. Pitch
     *       is <em>not</em> wrapped: pitch does not wrap, it stops, so folding a huge pitch delta into
     *       a plausible small one would hide a broken client rather than clamp it.
     *   <li>A negative zero never leaves here. {@code −0.0f} is legitimately produced by the rotation
     *       and by negating a zero delta, and it makes an otherwise centred {@link ControlInput}
     *       compare unequal to a centred one, since a record's {@code equals} uses
     *       {@code Float.compare}.
     * </ul>
     *
     * @param track the look memory from the previous packet; {@link LookTrack#UNSET} on launch
     * @param sample this packet's raw numbers, in the conventions {@link PilotInputSample} documents
     * @param dt seconds since the previous call; must be finite and positive. Unlike the sample this
     *     comes from our own tick loop, so a bad value is our bug and throws.
     */
    public PilotInputUpdate map(LookTrack track, PilotInputSample sample, double dt) {
        requirePresent(track, "track");
        requirePresent(sample, "sample");
        requireUsableDt(dt);

        double forward = 0.0;
        double lateral = 0.0;
        double frameYaw = resolveFrameYaw(sample, track);
        if (Double.isFinite(frameYaw)) {
            double sin = Math.sin(frameYaw);
            double cos = Math.cos(frameYaw);
            double wishX = finiteOrZero(sample.wishX());
            double wishZ = finiteOrZero(sample.wishZ());
            forward = -(sin * wishX + cos * wishZ) / this.mapping.wishFullScale();
            lateral = (cos * wishX - sin * wishZ) / this.mapping.wishFullScale();
        }

        double rollStick = 0.0;
        double pitchStick = 0.0;
        LookTrack nextTrack;
        if (sample.hasLook()) {
            if (track.present()) {
                double elapsed = track.secondsSinceSample() + dt;
                double yawDelta = wrapToPi(sample.lookYaw() - track.yaw());
                double pitchDelta = sample.lookPitch() - track.pitch();
                rollStick = -(yawDelta / elapsed) / this.mapping.rollLookRateFullScale();
                pitchStick = -(pitchDelta / elapsed) / this.mapping.pitchLookRateFullScale();
            }
            nextTrack = LookTrack.at(sample.lookYaw(), sample.lookPitch());
        } else {
            nextTrack = track.aged(dt);
        }

        ControlInput input =
                ControlInput.clamped(
                        stick((forward + 1.0) / 2.0),
                        stick(rollStick),
                        stick(pitchStick),
                        stick(lateral));
        return new PilotInputUpdate(input, nextTrack);
    }

    /**
     * The answer for a tick that received no packet at all: sticks centred, throttle resting at
     * mid-stick, and the look memory aged by the tick so the next real sample still divides by the
     * interval it actually spanned.
     *
     * <p>Mid-stick rather than closed for the same reason the remap is bipolar — it is where a
     * spring-centred throttle sits. A pilot whose client has gone quiet gets the drone's last
     * commanded attitude and a neutral throttle, which is a survivable failure; motors-off is not.
     */
    public PilotInputUpdate centred(LookTrack track, double dt) {
        requirePresent(track, "track");
        requireUsableDt(dt);
        return new PilotInputUpdate(new ControlInput(0.5f, 0f, 0f, 0f), track.aged(dt));
    }

    /**
     * The frame the wish vector was rotated into, or {@code NaN} when no source names it. Preferring
     * the sample's own fields over the track keeps the answer as fresh as the packet allows.
     */
    private static double resolveFrameYaw(PilotInputSample sample, LookTrack track) {
        if (Double.isFinite(sample.wishFrameYaw())) {
            return sample.wishFrameYaw();
        }
        if (Double.isFinite(sample.lookYaw())) {
            return sample.lookYaw();
        }
        return track.present() ? track.yaw() : Double.NaN;
    }

    /**
     * Folds an angle into {@code (−π, π]}. {@link Math#IEEEremainder} lands in the closed
     * {@code [−π, π]} and breaks halfway ties to even, which maps both {@code −π} and {@code 3π} onto
     * {@code −π} — a 540° left flick would come out banking right without the correction.
     */
    private static double wrapToPi(double radians) {
        if (!Double.isFinite(radians)) {
            return 0.0;
        }
        double wrapped = Math.IEEEremainder(radians, TWO_PI);
        return wrapped <= -Math.PI ? wrapped + TWO_PI : wrapped;
    }

    private static float stick(double value) {
        float narrowed = (float) value;
        return narrowed == 0f ? 0f : narrowed;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static void requireUsableDt(double dt) {
        if (!Double.isFinite(dt) || dt <= 0.0) {
            throw new IllegalArgumentException("dt must be finite and positive but was " + dt);
        }
    }

    private static void requirePresent(Object value, String name) {
        if (value == null) {
            throw new IllegalArgumentException(name + " must not be null");
        }
    }
}
