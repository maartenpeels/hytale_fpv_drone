package com.maartenpeels.fpv.plugin.camera;

import javax.annotation.Nonnull;

/**
 * Everything about the FPV camera a human might want to change while flying, as one validated
 * value.
 *
 * <p>It is a value rather than a read of {@code FpvConfig} because the diagnostic
 * {@code /fpv camera set} command overrides it at runtime: #19's whole problem is that only a human
 * can tell whether the camera is right, so every knob that might be the difference between "right"
 * and "wrong" has to be reachable without a rebuild.
 *
 * @param view whether the client tracks the drone or the server pushes attitude; see
 *     {@link DroneCameraView}
 * @param positionLerpSpeed client-side position smoothing. The protocol default is {@code 1.0};
 *     all three in-tree camera commands use {@code 0.2}
 *     (`server/core/command/commands/player/camera/PlayerCameraTopdownCommand.java:41-42`). At
 *     30 TPS this is a real candidate for the difference between "floaty" and "connected", which
 *     is why #19 exposes it instead of hard-coding it — #24's verdict depends on knowing whether
 *     it helped.
 * @param rotationLerpSpeed the same lever for rotation. More likely than the position one to
 *     matter for roll, since roll is the fastest-changing angle on a quad.
 * @param locked the packet's {@code isLocked} flag. All three in-tree camera commands lock, so
 *     {@code true} is the precedent — but locking is also the likeliest thing to kill the look
 *     channel #17's input adapter reads, and that failure looks like dead sticks rather than a
 *     dead camera. First knob to flip when the sticks go dead.
 * @param eyeHeight metres added to the camera's Y, so the view sits on the airframe rather than
 *     inside the mesh. Default matches the {@code EyeHeight: 0.1} #18 authored into the drone
 *     model (`docs/plans/18.md:233`). Applied by us, not by the client's {@code eyeOffset} flag —
 *     see {@link DroneCameraPackets} for why both would double up.
 * @param cameraUptiltRadians camera tilt, in <strong>radians</strong>, positive nose-up in Hytale's
 *     convention. Real FPV quads mount the camera tilted up so the pilot sees where a
 *     nose-down-and-accelerating airframe is going. Defaults to zero so that it cannot confuse the
 *     first flight test — a nonzero default would look exactly like a pitch sign error.
 */
public record DroneCameraTuning(
        @Nonnull DroneCameraView view,
        float positionLerpSpeed,
        float rotationLerpSpeed,
        boolean locked,
        double eyeHeight,
        float cameraUptiltRadians) {

    /** Hytale's own camera commands all use this; the protocol default is {@code 1.0f}. */
    public static final float DEFAULT_LERP_SPEED = 0.2f;

    /** Matches the drone model's authored {@code EyeHeight}. */
    public static final double DEFAULT_EYE_HEIGHT = 0.1;

    /**
     * What the plugin flies with until someone changes the config.
     *
     * <p>{@link DroneCameraView#TRACKED} is the default because it is the mode the ticket asks for
     * and the one that costs no per-tick bandwidth. {@link DroneCameraView#DRIVEN} is the fallback
     * if {@code AttachedToType.EntityId} turns out to be unimplemented client-side.
     */
    @Nonnull
    public static final DroneCameraTuning DEFAULT = new DroneCameraTuning(
            DroneCameraView.TRACKED,
            DEFAULT_LERP_SPEED,
            DEFAULT_LERP_SPEED,
            true,
            DEFAULT_EYE_HEIGHT,
            0.0f);

    /**
     * Validates on construction, because every one of these fields has a value that fails
     * <em>silently</em> — and a silently wrong camera is precisely what this ticket cannot detect
     * automatically.
     *
     * <p>A {@code NaN} lerp speed or uptilt is the worst case: it renders as no camera movement or
     * no tilt at all, which reads as "the client ignores this setting" and would send someone
     * hunting a protocol bug that is not there. That exact false negative nearly derailed #28,
     * where a degrees-for-radians mistake read as "roll is ignored".
     */
    public DroneCameraTuning {
        if (view == null) {
            throw new IllegalArgumentException("view must not be null");
        }
        requireFinite(positionLerpSpeed, "positionLerpSpeed");
        requireFinite(rotationLerpSpeed, "rotationLerpSpeed");
        requireFinite(cameraUptiltRadians, "cameraUptiltRadians");
        if (!Double.isFinite(eyeHeight)) {
            throw new IllegalArgumentException("eyeHeight must be finite, was " + eyeHeight);
        }
        if (positionLerpSpeed <= 0.0f) {
            throw new IllegalArgumentException(
                    "positionLerpSpeed must be positive, was " + positionLerpSpeed);
        }
        if (rotationLerpSpeed <= 0.0f) {
            throw new IllegalArgumentException(
                    "rotationLerpSpeed must be positive, was " + rotationLerpSpeed);
        }
    }

    private static void requireFinite(float value, @Nonnull String name) {
        if (!Float.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, was " + value);
        }
    }

    /** This tuning with a different view, for {@code /fpv camera set --view}. */
    @Nonnull
    public DroneCameraTuning withView(@Nonnull DroneCameraView newView) {
        return new DroneCameraTuning(
                newView,
                this.positionLerpSpeed,
                this.rotationLerpSpeed,
                this.locked,
                this.eyeHeight,
                this.cameraUptiltRadians);
    }

    /** This tuning with different smoothing, for {@code /fpv camera set --poslerp/--rotlerp}. */
    @Nonnull
    public DroneCameraTuning withLerpSpeeds(float position, float rotation) {
        return new DroneCameraTuning(
                this.view, position, rotation, this.locked, this.eyeHeight, this.cameraUptiltRadians);
    }

    /** This tuning with a different lock flag, for {@code /fpv camera set --locked}. */
    @Nonnull
    public DroneCameraTuning withLocked(boolean newLocked) {
        return new DroneCameraTuning(
                this.view,
                this.positionLerpSpeed,
                this.rotationLerpSpeed,
                newLocked,
                this.eyeHeight,
                this.cameraUptiltRadians);
    }

    /** This tuning with a different camera tilt, in radians. */
    @Nonnull
    public DroneCameraTuning withCameraUptiltRadians(float radians) {
        return new DroneCameraTuning(
                this.view,
                this.positionLerpSpeed,
                this.rotationLerpSpeed,
                this.locked,
                this.eyeHeight,
                radians);
    }
}
