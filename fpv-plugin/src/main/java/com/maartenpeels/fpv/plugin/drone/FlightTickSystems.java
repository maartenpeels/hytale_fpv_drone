package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.ArchetypeChunk;
import com.hypixel.hytale.component.CommandBuffer;
import com.hypixel.hytale.component.Holder;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.RemoveReason;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.component.SystemGroup;
import com.hypixel.hytale.component.dependency.Dependency;
import com.hypixel.hytale.component.query.Query;
import com.hypixel.hytale.component.system.HolderSystem;
import com.hypixel.hytale.component.system.RefSystem;
import com.hypixel.hytale.component.system.tick.EntityTickingSystem;
import com.hypixel.hytale.logger.HytaleLogger;
import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.flight.BodyRates;
import com.maartenpeels.fpv.flight.DroneState;
import com.maartenpeels.fpv.flight.FlightState;
import com.maartenpeels.fpv.flight.FlightTick;
import com.maartenpeels.fpv.flight.SubstepListener;
import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;
import com.maartenpeels.fpv.plugin.input.PilotInputBuffer;
import com.maartenpeels.fpv.plugin.input.PilotInputSlot;
import org.joml.Vector3d;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Set;
import java.util.UUID;
import java.util.logging.Level;

/**
 * The systems that make a drone fly: seed its flight state, own its input slot, and step the
 * integrator once per world tick.
 *
 * <p>None of them touch {@code World}, so all three run inside {@code HytaleEcsHarness}. Keeping that
 * true is what the {@link SubstepListener} injection in {@link AdvanceFlight} is for — see there.
 */
public final class FlightTickSystems {

    private FlightTickSystems() {}

    /**
     * Gives every drone a {@link DroneFlight}, seeded from the transform it spawned with.
     *
     * <p>A system rather than two more lines in {@code FlightSessions.launch}, following
     * {@link DroneLifecycleSystems.EnsureDroneNetworkSendable}'s precedent: a drone that exists but
     * carries no flight state would hang motionless in the air, and that is not a guarantee that
     * should depend on a caller remembering a line. Five builtin systems use the same idiom, listed in
     * {@code docs/plans/18.md}.
     *
     * <p>Runs inside {@code Store.addEntity}, before the entity has a {@code Ref}
     * (`component/Store.java:403-412`), which is precisely why it can read the pilot's
     * {@code UUIDComponent} through the store and be sure the pilot is still alive — on the removal
     * path it would routinely not be.
     */
    public static final class EnsureDroneFlight extends HolderSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final Query<EntityStore> query;

        public EnsureDroneFlight(@Nonnull FlightComponentTypes types) {
            this.types = types;
            this.query =
                    Query.and(types.drone(), types.transform(), Query.not(types.droneFlight()));
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.query;
        }

        @Override
        public void onEntityAdd(
                @Nonnull Holder<EntityStore> holder,
                @Nonnull AddReason reason,
                @Nonnull Store<EntityStore> store) {

            DroneComponent drone = holder.getComponent(this.types.drone());
            TransformComponent transform = holder.getComponent(this.types.transform());
            if (drone == null || transform == null) {
                return;
            }

            Ref<EntityStore> pilot = drone.getPilot();
            if (!pilot.isValid()) {
                return;
            }
            UUIDComponent pilotUuid = store.getComponent(pilot, this.types.uuid());
            if (pilotUuid == null) {
                // Not a player. Player entities always carry UUIDComponent -- Player.saveConfig
                // asserts it (`server/core/entity/entities/Player.java:303-305`) -- so this means a
                // drone was spawned for something that cannot supply input. Leaving DroneFlight off
                // is the honest outcome: the drone exists and does not fly.
                return;
            }

            Vector3d position = transform.getPosition();
            // Heading only. Pitch and roll are dropped so a drone never arms already banked, and the
            // pilot's own head pitch has nothing to do with the airframe's attitude.
            Quat heading = DroneRotation.headingOf(transform.getRotation());
            DroneState resting =
                    new DroneState(
                            new Vec3(position.x, position.y, position.z),
                            Vec3.ZERO,
                            heading,
                            BodyRates.ZERO);

            holder.putComponent(
                    this.types.droneFlight(),
                    new DroneFlight(pilotUuid.getUuid(), FlightState.beginning(resting)));
        }

        @Override
        public void onEntityRemoved(
                @Nonnull Holder<EntityStore> holder,
                @Nonnull RemoveReason reason,
                @Nonnull Store<EntityStore> store) {
            // Nothing to do: a drone's components die with its detached Holder.
        }
    }

    /**
     * Opens the pilot's input slot when a drone appears and closes it when the drone goes.
     *
     * <p>Queries {@link DroneFlight} rather than {@link DroneComponent} for two reasons that both come
     * down to the removal path. {@code DroneFlight} carries the pilot's {@code UUID}, so closing the
     * slot does not depend on the pilot's {@code Ref} still being valid — and it routinely is not,
     * because {@code Store.removeEntity} invalidates the pilot ref (`component/Store.java:666-681`)
     * before consuming the buffered drone removal (`:686`). And {@code HolderSystem}s have already run
     * by the time a {@code RefSystem} sees an entity being added (`Store.java:403-435`), so
     * {@link EnsureDroneFlight}'s component is guaranteed to be there.
     *
     * <p>Closing on removal is what keeps {@link PilotInputBuffer} bounded: presence of a key means
     * "this pilot is flying", and this system is the only thing that decides when that stops being
     * true.
     */
    public static final class TrackPilotInput extends RefSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final PilotInputBuffer inputs;

        public TrackPilotInput(
                @Nonnull FlightComponentTypes types, @Nonnull PilotInputBuffer inputs) {
            this.types = types;
            this.inputs = inputs;
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.types.droneFlight();
        }

        @Override
        public void onEntityAdded(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull AddReason reason,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            DroneFlight flight = commandBuffer.getComponent(ref, this.types.droneFlight());
            if (flight != null) {
                this.inputs.open(flight.getPilotId());
            }
        }

        @Override
        public void onEntityRemove(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull RemoveReason reason,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            DroneFlight flight = commandBuffer.getComponent(ref, this.types.droneFlight());
            if (flight != null) {
                this.inputs.close(flight.getPilotId());
            }
        }
    }

    /**
     * Runs the flight model once per world tick and writes the result onto the drone entity.
     *
     * <h2>What one tick does</h2>
     *
     * Drain the pilot's input slot for a single {@link ControlInput}, hand it to {@link FlightTick},
     * which runs the configured number of integration substeps over one tick's simulated time, and
     * copy the resulting position and attitude into {@code TransformComponent} and
     * {@code HeadRotation}.
     *
     * <p>Nothing else has to be told the drone moved. {@code NetworkId} plus
     * {@code TransformComponent} is what makes an entity network-sendable
     * (`modules/entity/system/NetworkSendableSpatialSystem.java:17`) and #18 guarantees both;
     * {@code TransformSystems.EntityTrackerUpdate} then diffs the transform against
     * {@code getSentTransform()} each tick and queues a {@code TransformUpdate} for every viewer
     * (`modules/entity/system/TransformSystems.java:61-92`).
     *
     * <h2>Why this stays testable despite writing a transform</h2>
     *
     * CLAUDE.md's second ECS convention says a system that mixes computation with world or network
     * side effects is not unit-testable. Writing a transform is neither, and that was verified rather
     * than assumed: {@code TransformComponent.setPosition}/{@code setRotation} are plain field
     * assignments (`modules/entity/component/TransformComponent.java:65,89`). The one world-touching
     * method on that class is {@code markChunkDirty} (`:133-140`), and it is deliberately not called —
     * a drone carries {@code NonSerialized}, so there is no chunk save for it to dirty.
     *
     * <p>The real world reach in this feature is terrain collision, which has to read blocks. That is
     * why it enters through an injected {@link SubstepListener} rather than being written inline: the
     * world-touching half lives in a collaborator, and tests inject {@link SubstepListener#NONE}. See
     * the field's comment for what #21 should do with it.
     *
     * <h2>Ordering</h2>
     *
     * Must run <em>before</em> {@code TransformSystems.EntityTrackerUpdate}, or every position reaches
     * the client one tick late — 33 ms added at 30 TPS to a feature whose only open question is whether
     * the latency is bearable. The group and dependencies are constructor parameters because
     * {@code SystemDependency.validate} throws for a system class the registry does not have
     * (`component/dependency/SystemDependency.java:31-35`) and the harness registry has no Hytale
     * systems. {@code DroneLifecycleSystems.HideParkedBody} takes the same shape for the same reason.
     */
    public static final class AdvanceFlight extends EntityTickingSystem<EntityStore> {

        /**
         * The sticks a drone flies on when its slot has gone missing — which should not happen, since
         * {@link TrackPilotInput} opens one for every drone. Mid-throttle, matching
         * {@link PilotInputMapper#centred}'s policy: a spring-centred throttle rests at mid-stick, and
         * cutting the motors is the worst available response to an internal inconsistency.
         */
        private static final ControlInput NO_SLOT = new ControlInput(0.5f, 0f, 0f, 0f);

        /** Beyond this factor between the observed tick and the configured one, something is wrong. */
        private static final double TICK_MISMATCH_FACTOR = 2.0;

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final FlightTick flightTick;
        @Nonnull
        private final PilotInputBuffer inputs;
        @Nonnull
        private final PilotInputMapper mapper;
        /**
         * The configured tick length, not the wall clock's.
         *
         * <p>{@code tick}'s {@code dt} is real elapsed seconds — {@code TickingThread.run} computes
         * {@code (float) delta / 1.0E9F} from {@code System.nanoTime()}
         * (`server/core/util/thread/TickingThread.java:54-72`) and {@code World.tick} passes it
         * straight through (`universe/world/World.java:342-351`) — so it grows whenever the server is
         * behind. Using it would make a GC pause or a debugger breakpoint hand the integrator a
         * multi-second step, which tunnels through terrain no matter how many substeps there are, and
         * would make flight irreproducible from {@code (state, input)}. Decision 3 says "N
         * <em>fixed</em> substeps per tick"; this is that word.
         */
        private final double tickSeconds;
        /**
         * Where #21 plugs terrain collision in, per substep.
         *
         * <p>{@link SubstepListener#NONE} until then. If collision needs per-drone state — a crash
         * verdict, an impact normal — widen this field to a per-drone factory; nothing else in the
         * plugin references it, so that is a one-line change here.
         */
        @Nonnull
        private final SubstepListener substepListener;
        @Nonnull
        private final HytaleLogger logger;
        @Nullable
        private final SystemGroup<EntityStore> group;
        @Nonnull
        private final Set<Dependency<EntityStore>> dependencies;
        @Nonnull
        private final Query<EntityStore> query;

        /**
         * Reused across drones and ticks so the hot loop allocates nothing.
         *
         * <p>Safe only because {@link #isParallel} is false — see there. Every drone's attitude passes
         * through this one object, so a parallel tick would tear one drone's rotation into another's.
         */
        @Nonnull
        private final Rotation3f scratchRotation = new Rotation3f();

        private boolean warnedTickMismatch;

        public AdvanceFlight(
                @Nonnull FlightComponentTypes types,
                @Nonnull FlightTick flightTick,
                @Nonnull PilotInputBuffer inputs,
                @Nonnull PilotInputMapper mapper,
                double tickSeconds,
                @Nonnull SubstepListener substepListener,
                @Nonnull HytaleLogger logger,
                @Nullable SystemGroup<EntityStore> group,
                @Nonnull Set<Dependency<EntityStore>> dependencies) {

            if (!Double.isFinite(tickSeconds) || tickSeconds <= 0.0) {
                throw new IllegalArgumentException(
                        "tickSeconds must be finite and positive but was " + tickSeconds);
            }
            this.types = types;
            this.flightTick = flightTick;
            this.inputs = inputs;
            this.mapper = mapper;
            this.tickSeconds = tickSeconds;
            this.substepListener = substepListener;
            this.logger = logger;
            this.group = group;
            this.dependencies = dependencies;
            this.query = Query.and(types.droneFlight(), types.transform());
        }

        @Nullable
        @Override
        public SystemGroup<EntityStore> getGroup() {
            return this.group;
        }

        @Nonnull
        @Override
        public Set<Dependency<EntityStore>> getDependencies() {
            return this.dependencies;
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.query;
        }

        /**
         * Never. Stated explicitly rather than inherited, because two pieces of this system's state are
         * shared across the entities it ticks — {@link #scratchRotation} and
         * {@link #warnedTickMismatch} — and neither is synchronised. The base class already returns
         * false ({@code component/system/tick/EntityTickingSystem.java:21-23}); this override exists so
         * that turning it on is a deliberate act with a comment to read first, not a one-word change.
         *
         * <p>Decision 9 puts the ceiling at 4–8 concurrent pilots, so there is nothing to gain: eight
         * drones times eight substeps is 64 integration steps a tick.
         */
        @Override
        public boolean isParallel(int archetypeChunkSize, int taskCount) {
            return false;
        }

        @Override
        public void tick(
                float dt,
                int index,
                @Nonnull ArchetypeChunk<EntityStore> archetypeChunk,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            this.warnOnceIfTickRateDisagreesWithConfig(dt);

            DroneFlight flight = archetypeChunk.getComponent(index, this.types.droneFlight());
            TransformComponent transform = archetypeChunk.getComponent(index, this.types.transform());
            if (flight == null || transform == null) {
                return;
            }

            ControlInput input = this.sticksFor(flight.getPilotId());
            FlightState next =
                    this.flightTick.advance(
                            flight.getState(), input, this.tickSeconds, this.substepListener);
            flight.setState(next);

            DroneState drone = next.drone();
            Vec3 position = drone.position();
            // getPosition().set(...) rather than setPosition(new Vector3d(...)): the accessor hands back
            // the component's own mutable vector (`TransformComponent.java:61-63`), so this writes in
            // place instead of allocating a Vector3d per drone per tick just to be copied out of.
            transform.getPosition().set(position.x(), position.y(), position.z());

            DroneRotation.writeTo(drone.orientation(), this.scratchRotation);
            transform.setRotation(this.scratchRotation);

            HeadRotation head = archetypeChunk.getComponent(index, this.types.headRotation());
            if (head != null) {
                // The drone has no separate head. Keeping them equal means the lookOrientation the
                // tracker sends (TransformSystems.EntityTrackerUpdate reads HeadRotation for it) agrees
                // with the body, instead of the airframe rolling while its "head" stays level.
                head.getRotation().set(this.scratchRotation);
            }
        }

        @Nonnull
        private ControlInput sticksFor(@Nonnull UUID pilotId) {
            PilotInputSlot slot = this.inputs.slotOf(pilotId);
            return slot == null ? NO_SLOT : slot.nextInput(this.mapper, this.tickSeconds);
        }

        /**
         * Says so, once, when the world is not ticking at the rate the config claims.
         *
         * <p>The cost of a fixed {@code tickSeconds} is that configuring {@code WorldTps} to 30 and
         * then running {@code /world tps 120} makes the drone fly at a quarter speed. That is exactly
         * the failure CLAUDE.md's world-units section describes — "not a crash, it is a drone that
         * flies wrong in a way nobody can point at" — so it gets a log line rather than a comment.
         *
         * <p>Deliberately not {@code World.setTps}: forcing a world-wide setting from a plugin is a
         * bigger act than reading one, and {@code /world tps} already exists.
         */
        private void warnOnceIfTickRateDisagreesWithConfig(float dt) {
            if (this.warnedTickMismatch || !Float.isFinite(dt) || dt <= 0f) {
                return;
            }
            double ratio = dt / this.tickSeconds;
            if (ratio > TICK_MISMATCH_FACTOR || ratio < 1.0 / TICK_MISMATCH_FACTOR) {
                this.warnedTickMismatch = true;
                // The speed factor is tickSeconds/dt, NOT dt/tickSeconds: each tick advances the sim
                // by tickSeconds, and the world delivers 1/dt ticks per real second, so simulated
                // seconds per real second is tickSeconds/dt. A world ticking slower than configured
                // makes the drone fly *slow*. Reporting the reciprocal would point an operator in
                // exactly the wrong direction, which is worse than saying nothing.
                this.logger
                        .at(Level.WARNING)
                        .log(
                                "FPV flight is simulating %.1f ticks/second (WorldTps in fpv_drone.json) but this "
                                        + "world is ticking at about %.1f. The drone will fly at %.2fx real speed "
                                        + "until the two agree -- set WorldTps to %.0f, or `/world tps %.0f`.",
                                1.0 / this.tickSeconds,
                                1.0 / dt,
                                this.tickSeconds / dt,
                                1.0 / dt,
                                1.0 / this.tickSeconds);
            }
        }
    }
}
