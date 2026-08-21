package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.Archetype;
import com.hypixel.hytale.component.ComponentRegistry;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.logger.HytaleLogger;
import com.hypixel.hytale.protocol.ToClientPacket;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.modules.entity.component.BoundingBox;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.EntityTrackerSystems;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.receiver.IPacketReceiver;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.control.PilotInputMapping;
import com.maartenpeels.fpv.flight.FlightTick;
import com.maartenpeels.fpv.flight.QuadParameters;
import com.maartenpeels.fpv.flight.SubstepListener;
import com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness;
import com.maartenpeels.fpv.plugin.input.PilotInputBuffer;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Set;
import java.util.UUID;

/**
 * Wires the drone feature into a {@link HytaleEcsHarness} the way {@code FPVDrone.setup()} wires
 * it into a real server.
 *
 * <p>This is the mirror image of the production wiring, and having both is the point of the
 * constructor-injection convention: {@code FPVDrone} resolves Hytale's component types through
 * the {@code SomeComponent.getComponentType()} statics, which need a booted server, while this
 * registers equivalents into a throwaway registry. Neither the systems nor {@link FlightSessions}
 * can tell the difference.
 *
 * @param types the component types, registered into the harness registry
 * @param sessions the service under test
 * @param inputs the input buffer the flight systems open and close slots in. Shared, exactly as
 *     {@code FPVDrone} shares one instance between its ECS systems and its packet watcher.
 */
public record DroneTestFixture(
        FlightComponentTypes types, FlightSessions sessions, PilotInputBuffer inputs) {

    /** Registers every component, then every lifecycle system, into the harness. */
    public static DroneTestFixture install(@Nonnull HytaleEcsHarness harness) {
        ComponentRegistry<EntityStore> registry = harness.registry();

        // Registration order matters: registerSystem validates a system's query against the
        // registry, so every queried component must already exist.
        FlightComponentTypes types = new FlightComponentTypes(
                registry.registerComponent(FlightSession.class, FlightSession.NO_DEFAULT),
                registry.registerComponent(ParkedBody.class, ParkedBody.ID, ParkedBody.CODEC),
                registry.registerComponent(DroneComponent.class, DroneComponent.NO_DEFAULT),
                registry.registerComponent(DroneFlight.class, DroneFlight.NO_DEFAULT),
                registry.registerComponent(TransformComponent.class, TransformComponent::new),
                registry.registerComponent(HeadRotation.class, HeadRotation::new),
                registry.registerComponent(UUIDComponent.class, UUIDComponent::randomUUID),
                registry.registerComponent(NetworkId.class, () -> new NetworkId(0)),
                registry.getNonSerializedComponentType(),
                registry.registerComponent(Invulnerable.class, () -> Invulnerable.INSTANCE),
                registry.registerComponent(Intangible.class, () -> Intangible.INSTANCE),
                registry.registerComponent(ModelComponent.class, () -> new ModelComponent(null)),
                registry.registerComponent(BoundingBox.class, BoundingBox::new),
                registry.registerComponent(
                        EntityTrackerSystems.EntityViewer.class, DroneTestFixture::newViewerComponent));

        registry.registerSystem(new DroneLifecycleSystems.EnsureDroneNetworkSendable(types));
        registry.registerSystem(new DroneLifecycleSystems.EndSessionOnPilotRemoved(types));
        registry.registerSystem(new DroneLifecycleSystems.ClearSessionOnDroneRemoved(types));
        registry.registerSystem(new DroneLifecycleSystems.RestoreParkedBodyOnAdd(types));
        // Null group and no dependencies: FIND_VISIBLE_ENTITIES_GROUP belongs to
        // EntityStore.REGISTRY and SystemGroup.validateRegistry would reject it here.
        registry.registerSystem(new DroneLifecycleSystems.HideParkedBody(types, null, Set.of()));

        // The flight lifecycle half of #23: seed DroneFlight on spawn, and own the input slot. Both
        // are registered here rather than per-test because they are part of what a drone *is* -- a
        // drone with no flight state hangs motionless, which no test should have to opt into.
        PilotInputBuffer inputs = new PilotInputBuffer();
        registry.registerSystem(new FlightTickSystems.EnsureDroneFlight(types));
        registry.registerSystem(new FlightTickSystems.TrackPilotInput(types, inputs));

        return new DroneTestFixture(types, new FlightSessions(types), inputs);
    }

    /**
     * Adds the ticking half of #23, which the shared {@link #install} deliberately leaves out so that
     * lifecycle tests are not silently flying drones around.
     *
     * <p>Null group and empty dependencies for the same reason as {@code HideParkedBody}:
     * {@code SystemDependency.validate} throws for a system class this registry does not have, and
     * {@code TransformSystems.EntityTrackerUpdate} is not registered here.
     */
    void installFlightTick(
            @Nonnull HytaleEcsHarness harness,
            @Nonnull FlightTick flightTick,
            double tickSeconds,
            @Nullable SubstepListener listener) {

        harness.registry().registerSystem(new FlightTickSystems.AdvanceFlight(
                this.types,
                flightTick,
                this.inputs,
                new PilotInputMapper(
                        PilotInputMapping.DEFAULT, QuadParameters.DEFAULT.hoverCollective()),
                tickSeconds,
                listener == null ? SubstepListener.NONE : listener,
                HytaleLogger.getLogger(),
                null,
                Set.of()));
    }

    /**
     * A stand-in pilot: a transform and a UUID. The real one also carries {@code Player} and
     * {@code PlayerRef}.
     *
     * <p>The UUID matters: it is the key the input slot is opened under, and a real player always has
     * one — {@code Player.saveConfig} asserts {@code UUIDComponent} non-null
     * (`server/core/entity/entities/Player.java:303-305`).
     */
    public Ref<EntityStore> newPilot(@Nonnull Store<EntityStore> store) {
        return store.addEntity(
                Archetype.of(this.types.transform(), this.types.uuid()), AddReason.SPAWN);
    }

    /** A pilot with no {@code UUIDComponent} — i.e. not a player, so no input can be routed to it. */
    Ref<EntityStore> newPilotWithoutUuid(@Nonnull Store<EntityStore> store) {
        return store.addEntity(Archetype.of(this.types.transform()), AddReason.SPAWN);
    }

    /** The pilot's account UUID, which is what {@code PilotInputBuffer} keys on. */
    UUID uuidOf(@Nonnull Store<EntityStore> store, @Nonnull Ref<EntityStore> pilot) {
        UUIDComponent component = store.getComponent(pilot, this.types.uuid());
        return component == null ? null : component.getUuid();
    }

    /** A viewer entity whose {@code visible} set the {@code HideParkedBody} system prunes. */
    Ref<EntityStore> newViewer(@Nonnull Store<EntityStore> store) {
        Ref<EntityStore> viewer = store.addEntity(Archetype.of(this.types.entityViewer()), AddReason.SPAWN);
        store.putComponent(viewer, this.types.entityViewer(), newViewerComponent());
        return viewer;
    }

    @Nonnull
    private static EntityTrackerSystems.EntityViewer newViewerComponent() {
        // HideParkedBody never touches packetReceiver, so a no-op sink is enough. Constructing a
        // real one would need a network channel, which is exactly what the harness avoids.
        return new EntityTrackerSystems.EntityViewer(16, new IPacketReceiver() {
            @Override
            public void write(@Nonnull ToClientPacket packet) {}

            @Override
            public void writeNoCache(@Nonnull ToClientPacket packet) {}
        });
    }
}
