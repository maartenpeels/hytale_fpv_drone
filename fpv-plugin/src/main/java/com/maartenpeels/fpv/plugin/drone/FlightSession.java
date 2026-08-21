package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.Component;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import java.util.function.Supplier;

/**
 * A pilot's live flight session, held on the <em>pilot's</em> entity.
 *
 * <p>This component <em>is</em> the session: a pilot is flying exactly when their entity carries
 * one. See {@link FlightSessions} for the ownership model and {@code docs/plans/18.md} for why a
 * plugin-side {@code Map<UUID, ...>} was rejected.
 *
 * <p><strong>Deliberately not serialized.</strong> Registered through the
 * {@code registerComponent(Class, Supplier)} overload, which
 * {@code ComponentRegistry.java:292-293} defines as no-id-no-codec, hence not persisted. A
 * session that survived a restart would point at a drone entity that did not.
 */
public final class FlightSession implements Component<EntityStore> {

    /**
     * Supplier for {@code registerComponent}, which requires one but never needs it here.
     *
     * <p>A session has no meaningful default — it only exists wrapped around a real drone ref.
     * The registry calls a component's supplier from exactly one place,
     * {@code Data.newComponent} (`component/ComponentRegistry.java:1567`), reached only via
     * {@code ensureComponent} and {@code addComponent(ref, type)}; this feature uses neither for
     * a session. Throwing keeps an accidental future {@code ensureComponent} loud rather than
     * silently creating a session that points at nothing.
     */
    @Nonnull
    public static final Supplier<FlightSession> NO_DEFAULT = () -> {
        throw new UnsupportedOperationException("FlightSession must be constructed around a drone ref");
    };

    @Nonnull
    private final Ref<EntityStore> drone;

    public FlightSession(@Nonnull Ref<EntityStore> drone) {
        this.drone = drone;
    }

    /**
     * The drone this pilot is flying.
     *
     * <p>May be invalid: a drone can be removed by something other than us (chunk unload,
     * {@code /entity remove}). {@code ClearSessionOnDroneRemoved} normally strips the session in
     * that case, so callers that see an invalid ref here are looking at a genuine bug — but
     * every mutation still goes through {@code tryRemove*}, which no-ops on an invalid ref.
     */
    @Nonnull
    public Ref<EntityStore> getDrone() {
        return this.drone;
    }

    @Nonnull
    @Override
    public Component<EntityStore> clone() {
        return new FlightSession(this.drone);
    }
}
