package com.maartenpeels;

import com.hypixel.hytale.component.ComponentRegistryProxy;
import com.hypixel.hytale.component.dependency.Dependency;
import com.hypixel.hytale.component.dependency.Order;
import com.hypixel.hytale.component.dependency.SystemDependency;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.modules.entity.component.BoundingBox;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.EntityTrackerSystems;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.plugin.JavaPlugin;
import com.hypixel.hytale.server.core.plugin.JavaPluginInit;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.hypixel.hytale.server.core.util.Config;
import com.maartenpeels.fpv.plugin.command.FpvCommand;
import com.maartenpeels.fpv.plugin.config.FpvConfig;
import com.maartenpeels.fpv.plugin.drone.DroneComponent;
import com.maartenpeels.fpv.plugin.drone.DroneLifecycleSystems;
import com.maartenpeels.fpv.plugin.drone.FlightComponentTypes;
import com.maartenpeels.fpv.plugin.drone.FlightSession;
import com.maartenpeels.fpv.plugin.drone.FlightSessions;
import com.maartenpeels.fpv.plugin.drone.ParkedBody;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Set;

/**
 * Plugin entry point. Must stay at this fully-qualified name — {@code main_class} in
 * {@code gradle.properties} points here and {@code manifest.json} is generated from it.
 *
 * <p>This class is an adapter and nothing more. Flight physics, rate curves, gate crossing
 * and race state live in {@code :fpv-core}; see CLAUDE.md decision 10.
 *
 * <p>It is also, deliberately, the only place in the plugin that calls
 * {@code SomeComponent.getComponentType()} — see CLAUDE.md's Conventions section.
 */
public class FPVDrone extends JavaPlugin {

    private final Config<FpvConfig> config;

    private FlightSessions flightSessions;

    public FPVDrone(@Nonnull JavaPluginInit init) {
        super(init);
        this.config = this.withConfig("fpv_drone", FpvConfig.CODEC);
    }

    @Override
    protected void setup() {
        this.config.save();
        this.flightSessions = installDroneFeature();
        this.getCommandRegistry().registerCommand(new FpvCommand(this));
    }

    /**
     * Registers the drone feature's components and lifecycle systems, and builds its service.
     *
     * <p>Ordering is load-bearing: components must be registered before the systems that query
     * them, because {@code registerSystem} validates a system's query against the registry before
     * registering that system's own component declarations.
     */
    @Nonnull
    private FlightSessions installDroneFeature() {
        ComponentRegistryProxy<EntityStore> registry = this.getEntityStoreRegistry();

        FlightComponentTypes types = new FlightComponentTypes(
                registry.registerComponent(FlightSession.class, FlightSession.NO_DEFAULT),
                registry.registerComponent(ParkedBody.class, ParkedBody.ID, ParkedBody.CODEC),
                registry.registerComponent(DroneComponent.class, DroneComponent.NO_DEFAULT),
                TransformComponent.getComponentType(),
                HeadRotation.getComponentType(),
                UUIDComponent.getComponentType(),
                NetworkId.getComponentType(),
                EntityStore.REGISTRY.getNonSerializedComponentType(),
                Invulnerable.getComponentType(),
                Intangible.getComponentType(),
                ModelComponent.getComponentType(),
                BoundingBox.getComponentType(),
                EntityTrackerSystems.EntityViewer.getComponentType());

        registry.registerSystem(new DroneLifecycleSystems.EnsureDroneNetworkSendable(types));
        registry.registerSystem(new DroneLifecycleSystems.EndSessionOnPilotRemoved(types));
        registry.registerSystem(new DroneLifecycleSystems.ClearSessionOnDroneRemoved(types));
        registry.registerSystem(new DroneLifecycleSystems.RestoreParkedBodyOnAdd(types));

        // The prune must run after CollectVisible has filled EntityViewer.visible, and inside the
        // group that owns that set. Both are passed in rather than resolved by the system, because
        // SystemGroup.validateRegistry rejects a group from another registry and a test uses its
        // own registry.
        Set<Dependency<EntityStore>> afterCollectVisible =
                Set.of(new SystemDependency<>(Order.AFTER, EntityTrackerSystems.CollectVisible.class));
        registry.registerSystem(new DroneLifecycleSystems.HideParkedBody(
                types, EntityTrackerSystems.FIND_VISIBLE_ENTITIES_GROUP, afterCollectVisible));

        return new FlightSessions(types);
    }

    @Nonnull
    public Config<FpvConfig> getFpvConfig() {
        return this.config;
    }

    /**
     * Launch and land flight sessions. {@code null} until {@code setup()} has run.
     *
     * <p>{@code /fpv launch} and {@code /fpv land} are #22; this is the API they will call.
     */
    @Nullable
    public FlightSessions getFlightSessions() {
        return this.flightSessions;
    }
}
