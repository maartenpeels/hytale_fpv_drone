package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.server.core.receiver.IPacketReceiver;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * Finds where a pilot's packets go.
 *
 * <p>This interface exists for one reason, and it is worth being blunt about it: <strong>#19 is
 * otherwise untestable end to end.</strong> The real answer is
 * {@code PlayerRef.getPacketHandler()}, and {@code PlayerRef} cannot be built in a
 * {@code HytaleEcsHarness} — its constructor needs a {@code PacketHandler} and a
 * {@code ChunkTracker}, i.e. a live connection, which is exactly what the harness exists to avoid
 * (and {@code PlayerRef.getComponentType()} additionally needs a booted {@code Universe}).
 *
 * <p>One interface between {@link DroneCamera} and the connection turns "did we send packet 280,
 * with the right drone id, at the right moment in the session lifecycle?" from a question only a
 * human can answer into an assertion. That question is where the silent regressions live: whether
 * the <em>client honours</em> the packet stays human-only regardless, so the least useful place to
 * lose test coverage is the part that is mechanical.
 *
 * @see PlayerRefSink the production implementation
 */
@FunctionalInterface
public interface PilotSink {

    /**
     * The pilot's packet sink, or {@code null} if they have none.
     *
     * <p>{@code null} is routine, not an error: a disconnecting pilot's session teardown runs after
     * their connection is gone, and a harness has no connections at all.
     */
    @Nullable
    IPacketReceiver receiverFor(
            @Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot);
}
