package com.maartenpeels.fpv.plugin.ecs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

// Individual imports, not a wildcard: `com.hypixel.hytale.component.system.System` would
// shadow `java.lang.System` for the whole file.
import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.Archetype;
import com.hypixel.hytale.component.ArchetypeChunk;
import com.hypixel.hytale.component.CommandBuffer;
import com.hypixel.hytale.component.Component;
import com.hypixel.hytale.component.ComponentType;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.component.query.Query;
import com.hypixel.hytale.component.system.tick.EntityTickingSystem;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Proof of life for {@link HytaleEcsHarness}, plus the two facts that constrain how every
 * plugin-side ECS system must be written.
 *
 * <p>These tests are the canary for the harness itself. The Hytale API is decompiled,
 * undocumented and pinned {@code >=0.5.3 <0.6.0}; if a future release stops letting the ECS run
 * standalone, this file fails and says which part broke.
 */
class HytaleEcsHarnessTest {

    private static final float TICK_DT = 1.0f / 30.0f;

    @Nested
    class DrivingTheEcsWithoutAServer {

        @Test
        void ticksARegisteredSystemOverAMatchingEntity() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                // Components first, then the system that queries them: registerSystem validates
                // the query against the registry before registering a system's own component
                // declarations, so a system whose query names an unregistered type is rejected.
                ComponentType<EntityStore, TickCounterComponent> counterType =
                        harness.registry().registerComponent(TickCounterComponent.class, TickCounterComponent::new);
                harness.registry().registerSystem(new TickCounterSystem(counterType));

                Store<EntityStore> store = harness.store();
                Ref<EntityStore> entity = store.addEntity(Archetype.of(counterType), AddReason.SPAWN);
                TickCounterComponent counter = store.getComponent(entity, counterType);

                assertEquals(0, counter.tickCount(), "system must not have run before the first tick");

                store.tick(TICK_DT);

                assertEquals(1, counter.tickCount(), "one tick must invoke the system exactly once");
                assertEquals(TICK_DT, counter.lastDt(), "dt must reach the system unmodified");
            }
        }

        @Test
        void accumulatesAcrossTicksSoSubsteppingCanBeTestedAsRepeatedSteps() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                ComponentType<EntityStore, TickCounterComponent> counterType =
                        harness.registry().registerComponent(TickCounterComponent.class, TickCounterComponent::new);
                harness.registry().registerSystem(new TickCounterSystem(counterType));

                Store<EntityStore> store = harness.store();
                Ref<EntityStore> entity = store.addEntity(Archetype.of(counterType), AddReason.SPAWN);
                TickCounterComponent counter = store.getComponent(entity, counterType);

                for (int i = 0; i < 8; i++) {
                    store.tick(TICK_DT / 8.0f);
                }

                assertEquals(8, counter.tickCount());
                assertEquals(TICK_DT / 8.0f, counter.lastDt());
            }
        }

        @Test
        void skipsEntitiesTheSystemsQueryDoesNotMatch() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                ComponentType<EntityStore, TickCounterComponent> counterType =
                        harness.registry().registerComponent(TickCounterComponent.class, TickCounterComponent::new);
                ComponentType<EntityStore, UnrelatedComponent> unrelatedType =
                        harness.registry().registerComponent(UnrelatedComponent.class, UnrelatedComponent::new);
                harness.registry().registerSystem(new TickCounterSystem(counterType));

                Store<EntityStore> store = harness.store();
                Ref<EntityStore> matched = store.addEntity(Archetype.of(counterType), AddReason.SPAWN);
                Ref<EntityStore> unmatched = store.addEntity(Archetype.of(unrelatedType), AddReason.SPAWN);

                store.tick(TICK_DT);

                assertEquals(
                        1,
                        store.getComponent(matched, counterType).tickCount(),
                        "the entity inside the query must still be ticked");
                assertNull(
                        store.getComponent(unmatched, counterType),
                        "an entity outside the query must not have gained the queried component");
            }
        }

        @Test
        void givesEachHarnessAFreshRegistrySoTestsCannotLeakIntoEachOther() {
            try (HytaleEcsHarness first = HytaleEcsHarness.create();
                    HytaleEcsHarness second = HytaleEcsHarness.create()) {
                assertNotSame(first.registry(), second.registry());
                assertNotSame(
                        EntityStore.REGISTRY,
                        first.registry(),
                        "never register into the shared static EntityStore.REGISTRY from a test");
            }
        }
    }

    /**
     * Why plugin systems take their {@code ComponentType} by constructor injection. If this
     * test ever fails, Hytale has changed and the convention in CLAUDE.md should be re-decided
     * deliberately rather than left as folklore.
     */
    @Nested
    class WhyComponentTypesMustBeInjected {

        @Test
        void staticGetComponentTypeAccessorsFailWithoutABootedServer() {
            // TransformComponent.getComponentType() resolves through EntityModule.get(), whose
            // static instance is only assigned in the module's plugin constructor. Outside a
            // booted server it is null.
            assertThrows(NullPointerException.class, TransformComponent::getComponentType);
        }
    }

    /** A trivial component: records that a system saw it, and with what dt. */
    static final class TickCounterComponent implements Component<EntityStore> {

        private int tickCount;
        private float lastDt;

        void recordTick(float dt) {
            tickCount++;
            lastDt = dt;
        }

        int tickCount() {
            return tickCount;
        }

        float lastDt() {
            return lastDt;
        }

        @Override
        public Component<EntityStore> clone() {
            TickCounterComponent copy = new TickCounterComponent();
            copy.tickCount = tickCount;
            copy.lastDt = lastDt;
            return copy;
        }
    }

    /** A second component, so query-matching can be tested with two distinct types. */
    static final class UnrelatedComponent implements Component<EntityStore> {

        @Override
        public Component<EntityStore> clone() {
            return new UnrelatedComponent();
        }
    }

    /**
     * The shape every plugin ECS system should copy: the {@link ComponentType} arrives through
     * the constructor, so the system never touches a module singleton and stays testable.
     */
    static final class TickCounterSystem extends EntityTickingSystem<EntityStore> {

        private final ComponentType<EntityStore, TickCounterComponent> counterType;

        TickCounterSystem(ComponentType<EntityStore, TickCounterComponent> counterType) {
            this.counterType = counterType;
        }

        @Override
        public Query<EntityStore> getQuery() {
            // ComponentType implements Query, so a single required component is its own query.
            return counterType;
        }

        @Override
        public void tick(
                float dt,
                int index,
                ArchetypeChunk<EntityStore> archetypeChunk,
                Store<EntityStore> store,
                CommandBuffer<EntityStore> commandBuffer) {
            archetypeChunk.getComponent(index, counterType).recordTick(dt);
        }
    }
}
