package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.Component;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import java.util.function.Supplier;

/**
 * Marks an entity as a drone and names its pilot.
 *
 * <p>Two jobs. It is the marker {@code EnsureDroneNetworkSendable} keys on to make the entity
 * client-visible and non-serialized, following the pattern of
 * {@code MountSystems.EnsureMinecartComponents} (`builtin/mounts/MountSystems.java:95-130`). And
 * it carries the pilot back-reference, which the flight tick (#23) needs to read control input
 * from, and which lets drone-side removal clear the pilot's {@link FlightSession} rather than
 * leaving a stale handle.
 *
 * <p>Named {@code DroneComponent} rather than {@code Drone} to match Hytale's own
 * {@code ModelComponent}/{@code TransformComponent} naming.
 *
 * <p>Not serialized — the entity as a whole carries {@code NonSerialized}, so persisting this
 * would be meaningless.
 */
public final class DroneComponent implements Component<EntityStore> {

    /**
     * Supplier for {@code registerComponent}. See {@link FlightSession#NO_DEFAULT} — a drone
     * without a pilot is not a state this feature has.
     */
    @Nonnull
    public static final Supplier<DroneComponent> NO_DEFAULT = () -> {
        throw new UnsupportedOperationException("DroneComponent must be constructed around a pilot ref");
    };

    @Nonnull
    private final Ref<EntityStore> pilot;

    public DroneComponent(@Nonnull Ref<EntityStore> pilot) {
        this.pilot = pilot;
    }

    /**
     * The pilot flying this drone.
     *
     * <p><strong>Check {@code isValid()} before use.</strong> When the pilot goes first,
     * {@code Store.removeEntity} invalidates their ref (`component/Store.java:666-681`) before
     * it consumes the buffered drone removal (`:686`), so a drone's removal callback routinely
     * sees a dead pilot ref.
     */
    @Nonnull
    public Ref<EntityStore> getPilot() {
        return this.pilot;
    }

    @Nonnull
    @Override
    public Component<EntityStore> clone() {
        return new DroneComponent(this.pilot);
    }
}
