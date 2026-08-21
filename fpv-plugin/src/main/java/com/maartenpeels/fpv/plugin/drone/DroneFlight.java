package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.Component;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.flight.FlightState;

import javax.annotation.Nonnull;
import java.util.UUID;
import java.util.function.Supplier;

/**
 * A drone's simulated flight: the {@link FlightState} the integrator carries forward, and whose input
 * to fly it on.
 *
 * <p>The state itself is {@code :fpv-core}'s immutable {@code (DroneState, RatePidState)} pair, so this
 * is a holder for a value that gets <em>replaced</em> each tick rather than mutated. That is what keeps
 * a tick a pure function of {@code (state, input, dt)} — decision 10's whole reason for existing — with
 * the impurity confined to one field assignment in one ECS component.
 *
 * <h2>Why the pilot's UUID is cached here</h2>
 *
 * {@link DroneComponent#getPilot()} already names the pilot, so this looks redundant. It is not, and
 * the reason is the drone's removal path. The pilot's input slot has to be closed when the drone dies,
 * and at that moment the pilot's {@code Ref} is routinely already invalid — {@code Store.removeEntity}
 * invalidates it (`component/Store.java:666-681`) before consuming the buffered drone removal (`:686`),
 * which is exactly what {@code DroneComponent.getPilot()}'s javadoc warns about. A {@code UUID} sitting
 * on the drone's own holder cannot go stale, so the close is unconditional rather than best-effort.
 *
 * <p>Not serialized: the drone entity as a whole carries {@code NonSerialized}, so persisting a
 * position that a restart invalidates would be meaningless.
 */
public final class DroneFlight implements Component<EntityStore> {

    /**
     * Supplier for {@code registerComponent}. Matches {@link FlightSession#NO_DEFAULT}: there is no
     * such thing as a drone flying no particular state for no particular pilot, and a silently
     * default-constructed one would sit at the origin.
     */
    @Nonnull
    public static final Supplier<DroneFlight> NO_DEFAULT = () -> {
        throw new UnsupportedOperationException(
                "DroneFlight must be constructed around a FlightState and a pilot id");
    };

    @Nonnull
    private final UUID pilotId;

    @Nonnull
    private FlightState state;

    public DroneFlight(@Nonnull UUID pilotId, @Nonnull FlightState state) {
        this.pilotId = pilotId;
        this.state = state;
    }

    /** The account UUID of the pilot whose input this drone flies on, and whose slot it owns. */
    @Nonnull
    public UUID getPilotId() {
        return this.pilotId;
    }

    @Nonnull
    public FlightState getState() {
        return this.state;
    }

    public void setState(@Nonnull FlightState state) {
        this.state = state;
    }

    @Nonnull
    @Override
    public Component<EntityStore> clone() {
        return new DroneFlight(this.pilotId, this.state);
    }
}
