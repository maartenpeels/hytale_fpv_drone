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
 * one. There is no plugin-side {@code Map<UUID, ...>}, and that is the whole point. Every way a
 * pilot can leave — disconnect, kick, world switch, world crash — funnels through
 * {@code PlayerRef.removeFromStore()} (`universe/PlayerRef.java:161`), which is a single
 * {@code Store.removeEntity(ref, RemoveReason.UNLOAD)}. So one {@code RefSystem} on this
 * component sees every exit, whereas a map has to be swept by hand on each one.
 *
 * <p>The codebase settles that argument against itself. {@code ParkourPlugin}'s
 * {@code Object2IntMap<UUID>} (`builtin/parkour/ParkourPlugin.java:27-28`) is only cleaned on
 * course completion and leaks permanently when a player disconnects mid-course;
 * {@code BuilderToolsPlugin} pays for its map with a scheduled sweeper, a retention timestamp
 * and a pair of connect/disconnect listeners. The same plugin's component-based state,
 * {@code BuilderToolsUserData}, needs none of it — the component rides the detached
 * {@code Holder} out of the store and is garbage-collected with it.
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
