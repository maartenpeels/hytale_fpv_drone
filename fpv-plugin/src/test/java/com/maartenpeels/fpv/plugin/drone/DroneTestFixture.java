package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.Archetype;
import com.hypixel.hytale.component.ComponentRegistry;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
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
import com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness;

import javax.annotation.Nonnull;
import java.util.Set;

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
 */
record DroneTestFixture(FlightComponentTypes types, FlightSessions sessions) {

    /** Registers every component, then every lifecycle system, into the harness. */
    static DroneTestFixture install(@Nonnull HytaleEcsHarness harness) {
        ComponentRegistry<EntityStore> registry = harness.registry();

        // Registration order matters: registerSystem validates a system's query against the
        // registry, so every queried component must already exist.
        FlightComponentTypes types = new FlightComponentTypes(
                registry.registerComponent(FlightSession.class, FlightSession.NO_DEFAULT),
                registry.registerComponent(ParkedBody.class, ParkedBody.ID, ParkedBody.CODEC),
                registry.registerComponent(DroneComponent.class, DroneComponent.NO_DEFAULT),
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

        return new DroneTestFixture(types, new FlightSessions(types));
    }

    /** A stand-in pilot: any entity with a transform. The real one also carries {@code Player}. */
    Ref<EntityStore> newPilot(@Nonnull Store<EntityStore> store) {
        return store.addEntity(Archetype.of(this.types.transform()), AddReason.SPAWN);
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
