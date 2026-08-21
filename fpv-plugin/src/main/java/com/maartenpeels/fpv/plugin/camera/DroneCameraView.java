package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.protocol.RotationType;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Locale;

/**
 * How the client is told to follow the drone. The two values are the two {@link RotationType}
 * constants, and they fail for independent reasons — which is the whole reason both ship.
 *
 * <p>#28 established that the client renders camera roll in <em>both</em> Hytale rotation modes.
 * What it could not establish is anything about {@code AttachedToType.EntityId}, because it
 * attached to the local player. That value has <strong>zero usages anywhere in the decompiled
 * server</strong> — all five in-tree {@code SetServerCamera} senders leave {@code attachedToType}
 * at its {@code LocalPlayer} default — and the decompiled cache contains no client tree, so its
 * interpretation is unreadable. It may simply be unimplemented.
 *
 * <p>{@link #TRACKED} depends on it entirely. {@link #DRIVEN} does not touch it. So a mode switch
 * turns an unknown that could block phase 0 into a config value.
 */
public enum DroneCameraView {

    /**
     * Client-side tracking: {@code attachedToType = EntityId} plus
     * {@code RotationType.AttachedToPlusOffset}.
     *
     * <p>One packet per session. The client follows the drone's own position and rotation, and roll
     * rides along inside {@code ModelTransform.bodyOrientation} — verified on the wire:
     * {@code TransformComponent.rotation.z} → {@code PositionUtil.assign}
     * (`server/core/util/PositionUtil.java:58-62`, which copies all three angles) →
     * {@code TransformUpdate} → {@code EntityUpdates} (packet 161).
     *
     * <p>Cheapest, and the shape the ticket names. Unproven end to end.
     */
    TRACKED(RotationType.AttachedToPlusOffset),

    /**
     * Server-driven: {@code RotationType.Custom} with the drone's full attitude, re-sent every
     * tick.
     *
     * <p>Uses only the path #28 actually proved, and touches neither {@code EntityId} nor offset
     * semantics. Costs one 157-byte packet per pilot per tick — ~4.7 kB/s at 30 TPS, ~38 kB/s at
     * decision 9's eight-pilot target.
     *
     * <p>Freezing the client's own mouse-driven camera yaw/pitch is correct here rather than a
     * limitation: under decision 2 the server sim owns all three angles, and the mouse is an input
     * channel into the sim (decision 1), not a direct camera control.
     */
    DRIVEN(RotationType.Custom);

    @Nonnull
    private final RotationType rotationType;

    DroneCameraView(@Nonnull RotationType rotationType) {
        this.rotationType = rotationType;
    }

    /** The Hytale rotation mode this view sends. */
    @Nonnull
    public RotationType rotationType() {
        return this.rotationType;
    }

    /** Whether this view needs re-sending every tick to follow the drone. */
    public boolean needsPerTickPush() {
        return this == DRIVEN;
    }

    /**
     * Parses a config or command value, case-insensitively, falling back rather than throwing.
     *
     * <p>Lenient on purpose: this is read from a hand-edited config file, and a typo that
     * hard-failed plugin setup would take the whole plugin down over a camera preference. A typo
     * that silently picks the wrong mode is visible the moment someone flies.
     *
     * @param fallback returned for {@code null}, blank or unrecognised input
     */
    @Nonnull
    public static DroneCameraView parse(@Nullable String value, @Nonnull DroneCameraView fallback) {
        DroneCameraView parsed = parseOrNull(value);
        return parsed == null ? fallback : parsed;
    }

    /**
     * Parses a value, or {@code null} if it names no view.
     *
     * <p>The strict counterpart to {@link #parse}. A command that silently ignored a typo would be
     * actively misleading: {@code /fpv camera set --view driven} exists to tell two failure modes
     * apart, so quietly staying in the other mode is the one outcome worse than an error.
     */
    @Nullable
    public static DroneCameraView parseOrNull(@Nullable String value) {
        if (value == null) {
            return null;
        }
        String normalised = value.trim().toUpperCase(Locale.ROOT);
        for (DroneCameraView candidate : values()) {
            if (candidate.name().equals(normalised)) {
                return candidate;
            }
        }
        return null;
    }
}
