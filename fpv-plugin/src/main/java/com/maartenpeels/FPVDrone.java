package com.maartenpeels;

import com.hypixel.hytale.component.ComponentRegistryProxy;
import com.hypixel.hytale.component.dependency.Dependency;
import com.hypixel.hytale.component.dependency.Order;
import com.hypixel.hytale.component.dependency.SystemDependency;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.io.adapter.PacketFilter;
import com.hypixel.hytale.server.core.modules.entity.component.BoundingBox;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.system.TransformSystems;
import com.hypixel.hytale.server.core.modules.entity.tracker.EntityTrackerSystems;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.plugin.JavaPlugin;
import com.hypixel.hytale.server.core.plugin.JavaPluginInit;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.hypixel.hytale.server.core.util.Config;
import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.control.PilotInputMapping;
import com.maartenpeels.fpv.flight.FlightTick;
import com.maartenpeels.fpv.flight.QuadIntegrator;
import com.maartenpeels.fpv.flight.QuadParameters;
import com.maartenpeels.fpv.flight.SubstepListener;
import com.maartenpeels.fpv.plugin.command.FpvCommand;
import com.maartenpeels.fpv.plugin.config.FpvConfig;
import com.maartenpeels.fpv.plugin.drone.DroneComponent;
import com.maartenpeels.fpv.plugin.drone.DroneFlight;
import com.maartenpeels.fpv.plugin.drone.DroneLifecycleSystems;
import com.maartenpeels.fpv.plugin.drone.FlightComponentTypes;
import com.maartenpeels.fpv.plugin.drone.FlightSession;
import com.maartenpeels.fpv.plugin.drone.FlightSessions;
import com.maartenpeels.fpv.plugin.drone.FlightTickSystems;
import com.maartenpeels.fpv.plugin.drone.ParkedBody;
import com.maartenpeels.fpv.plugin.input.ClientMovementWatcher;
import com.maartenpeels.fpv.plugin.input.PilotInputBuffer;

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

    /**
     * Where the netty thread leaves pilot input for the world thread to pick up. Created in the
     * constructor because both the ECS systems (built in {@code setup}) and the packet watcher (built
     * in {@code start}) need the same instance.
     */
    private final PilotInputBuffer pilotInputs = new PilotInputBuffer();

    private FlightSessions flightSessions;

    /**
     * The deregistration key for our inbound packet watcher.
     *
     * <p>Not the watcher — {@code PacketAdapters.registerInbound} wraps it in a {@code PacketFilter}
     * and only that wrapper can be removed (`server/core/io/adapter/PacketAdapters.java:63-72,90-96`),
     * and {@code deregisterInbound} throws for anything it does not recognise.
     */
    @Nullable
    private PacketFilter movementWatcher;

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
     * Subscribes to inbound packets, and nothing else.
     *
     * <p>This is one statement on purpose. {@code PacketAdapters}' handler lists are {@code static} and
     * JVM-global (`PacketAdapters.java:13-14`), so a registration that is never undone survives a hot
     * reload and runs alongside its replacement — two watchers filling the same input slots, which
     * would read as doubled look sensitivity rather than as a leak.
     *
     * <p>{@code start()}/{@code shutdown()} is the only exactly matched pair available:
     * {@code PluginBase.start0} reaches {@code start()} only when {@code setup()} succeeded and reaches
     * {@code ENABLED} only when {@code start()} returned (`server/core/plugin/PluginBase.java:259-274`),
     * and {@code PluginManager.shutdown} calls {@code shutdown0} only for an {@code ENABLED} plugin
     * (`server/core/plugin/PluginManager.java:398`). Registering in {@code setup()} instead would leak
     * the watcher permanently on any later setup failure.
     */
    @Override
    protected void start() {
        this.movementWatcher = new ClientMovementWatcher(this.pilotInputs).register();
    }

    @Override
    protected void shutdown() {
        ClientMovementWatcher.deregister(this.movementWatcher);
        this.movementWatcher = null;
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
                registry.registerComponent(DroneFlight.class, DroneFlight.NO_DEFAULT),
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

        installFlightTick(registry, types);
        return new FlightSessions(types);
    }

    /**
     * Registers the systems that actually fly the drone.
     *
     * <p>The two decision-3 knobs — substep count and world TPS — are read from config exactly once,
     * here. {@code FlightTick} then owns the arithmetic: it is handed one tick's duration and divides
     * it by the substep count, so total simulated time per tick is the same at any substep count and
     * "more substeps" costs CPU and changes nothing else. Raising {@code WorldTps} needs no code
     * change; changing either value needs a plugin reload, which is what a config value normally
     * means.
     */
    private void installFlightTick(
            @Nonnull ComponentRegistryProxy<EntityStore> registry,
            @Nonnull FlightComponentTypes types) {

        FpvConfig fpv = this.config.get();
        FlightTick flightTick =
                new FlightTick(new QuadIntegrator(QuadParameters.DEFAULT), fpv.getPhysicsSubsteps());

        registry.registerSystem(new FlightTickSystems.EnsureDroneFlight(types));
        registry.registerSystem(new FlightTickSystems.TrackPilotInput(types, this.pilotInputs));

        // Before the tracker's transform diff, or every position the client sees is a tick stale.
        // Passed in rather than resolved inside the system: SystemDependency.validate throws for a
        // system class the registry does not have, which would put this system out of the test
        // harness's reach. Same shape as HideParkedBody above.
        Set<Dependency<EntityStore>> beforeTrackerUpdate =
                Set.of(new SystemDependency<>(Order.BEFORE, TransformSystems.EntityTrackerUpdate.class));
        registry.registerSystem(new FlightTickSystems.AdvanceFlight(
                types,
                flightTick,
                this.pilotInputs,
                new PilotInputMapper(PilotInputMapping.DEFAULT),
                1.0 / fpv.getWorldTps(),
                // #21 replaces this with terrain collision, called per substep so a fast drone cannot
                // tunnel between two of them.
                SubstepListener.NONE,
                this.getLogger(),
                null,
                beforeTrackerUpdate));
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
