package com.maartenpeels.fpv.plugin.ecs;

// Trap: `com.hypixel.hytale.component.system.System` collides with `java.lang.System`. Import
// Hytale system classes individually -- a wildcard import here breaks every `System.out` in
// the file.
import com.hypixel.hytale.component.ComponentRegistry;
import com.hypixel.hytale.component.EmptyResourceStorage;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

/**
 * Drives Hytale's ECS in a plain JVM -- no server boot, no {@code Assets.zip}, no client.
 *
 * <p>This exists so {@code :fpv-plugin}'s systems can be unit-tested. {@link ComponentRegistry}
 * has a public no-arg constructor that pulls in no world, no scheduler, no network and no
 * assets, and {@link Store#tick(float)} is public, so a registry plus a store is all a test
 * needs to run a system for one tick.
 *
 * <p><strong>Requires a JVM flag.</strong> Every test using this class must run on a JVM
 * started with
 * {@code -Djava.util.logging.manager=com.hypixel.hytale.logger.backend.HytaleLogManager}.
 * {@code ComponentRegistry} holds a {@code static final HytaleLogger}, whose static
 * initialiser throws {@code IllegalStateException: Log manager wasn't set!} otherwise.
 * Setting the system property from inside a test is too late. {@code :fpv-plugin}'s Gradle
 * {@code test} task passes it; a hand-rolled runner must too.
 *
 * <p><strong>Single-threaded only.</strong> A {@link Store} binds to the thread that
 * constructs it and throws {@code IllegalStateException} if touched from another, so create
 * and use a harness on one thread, and never enable JUnit parallel execution for these tests.
 *
 * <p><strong>Each instance gets a fresh registry.</strong> Never register into
 * {@code EntityStore.REGISTRY}: it is {@code public static final} and shared JVM-wide, so
 * re-registering an id throws {@code id 'Transform' already exists!} and leaks state between
 * tests. Component registration is per-registry-instance, which is what makes isolation work.
 *
 * <p><strong>The world is {@code null}, deliberately.</strong> The ECS type parameter is
 * {@link EntityStore} so that harness-driven tests can instantiate the very systems the plugin
 * ships -- they are all {@code <EntityStore>}-typed. {@code EntityStore}'s constructor only
 * assigns its {@code World} field, so a {@code null} world is a valid external-data object for
 * any system that does not reach for the world. If a system under test does reach for it, the
 * test fails with a {@link NullPointerException} out of {@code EntityStore.getWorld()}. That is
 * the harness reporting a design problem, not a harness defect: a system that mixes
 * computation with world, chunk or network side effects is not unit-testable, and should be
 * split so the computational half is.
 *
 * <p>Usage -- register components first, then the systems that query them, because
 * {@code registerSystem} validates a system's query before registering the system's own
 * component declarations:
 *
 * <pre>{@code
 * try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
 *     var type = harness.registry().registerComponent(MyComponent.class, MyComponent::new);
 *     harness.registry().registerSystem(new MySystem(type));
 *
 *     Ref<EntityStore> entity = harness.store().addEntity(Archetype.of(type), AddReason.SPAWN);
 *     harness.store().tick(1.0f / 30.0f);
 * }
 * }</pre>
 */
public final class HytaleEcsHarness implements AutoCloseable {

    private final ComponentRegistry<EntityStore> registry;
    private final Store<EntityStore> store;
    private boolean closed;

    private HytaleEcsHarness(ComponentRegistry<EntityStore> registry, Store<EntityStore> store) {
        this.registry = registry;
        this.store = store;
    }

    /**
     * Creates a registry and a store bound to the calling thread.
     *
     * <p>Registering components and systems after the store exists is fine:
     * {@code Store.tick} calls {@code registry.doDataUpdate()} first, which flushes pending
     * registrations.
     */
    public static HytaleEcsHarness create() {
        ComponentRegistry<EntityStore> registry = new ComponentRegistry<>();
        // A null World: see the class javadoc. EntityStore's constructor only assigns the field.
        EntityStore externalData = new EntityStore(null);
        Store<EntityStore> store = registry.addStore(externalData, EmptyResourceStorage.get());
        return new HytaleEcsHarness(registry, store);
    }

    /** The per-harness registry. Register components before the systems that query them. */
    public ComponentRegistry<EntityStore> registry() {
        return registry;
    }

    /** The store, bound to the thread that called {@link #create()}. */
    public Store<EntityStore> store() {
        return store;
    }

    /**
     * Shuts the registry down, interrupting the daemon thread every registry starts. Always
     * call this -- {@code try}-with-resources does it for you.
     */
    @Override
    public void close() {
        if (!closed) {
            closed = true;
            registry.shutdown();
        }
    }
}
