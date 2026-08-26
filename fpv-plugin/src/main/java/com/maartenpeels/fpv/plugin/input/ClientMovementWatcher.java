package com.maartenpeels.fpv.plugin.input;

import com.hypixel.hytale.protocol.Packet;
import com.hypixel.hytale.server.core.io.adapter.PacketAdapters;
import com.hypixel.hytale.server.core.io.adapter.PacketFilter;
import com.hypixel.hytale.server.core.io.adapter.PlayerPacketWatcher;
import com.hypixel.hytale.protocol.packets.player.ClientMovement;
import com.hypixel.hytale.server.core.universe.PlayerRef;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Reads packet 108 off the wire and drops it in the flying pilot's input slot.
 *
 * <p>This is the production caller decision 1 always implied and nothing had yet supplied: #17 built
 * the whole mapping path and stopped at "a {@code ClientMovement} in hand becomes a
 * {@code PilotInputSample}", so until now {@code :fpv-core}'s input mapping was exercised only by its
 * own tests.
 *
 * <h2>The hook</h2>
 *
 * {@code PacketAdapters.registerInbound} (`server/core/io/adapter/PacketAdapters.java:63-72`) is a
 * public static registry, and {@code PlayerChannelHandler.channelRead} calls
 * {@code PacketAdapters.__handleInbound} for every inbound packet <em>before</em> dispatching it
 * (`server/core/io/netty/PlayerChannelHandler.java:30`). It is not on {@code JavaPlugin}, which is
 * why #17 read that 34-line class, found no packet API, and concluded there was no hook.
 *
 * <p>The {@code PlayerPacketWatcher} overload is used rather than a filter because a filter returning
 * {@code true} <b>swallows the packet</b> (`:99-101`) — and packet 108 must keep reaching
 * {@code GamePacketHandler}, since the pilot's own body still needs it, not least for the chunk
 * streaming #20 depends on. The watcher wrappers always return {@code false}.
 *
 * <h2>Four constraints, and how each is met here</h2>
 *
 * <ul>
 *   <li><b>Netty thread.</b> This class touches no ECS, reads no component and holds no {@code Ref}.
 *       It reads {@code PlayerRef.getUuid()} — a final field — and writes one reference into a
 *       {@link PilotInputBuffer}.
 *   <li><b>Exceptions are swallowed and logged at SEVERE</b> (`:104-106`), so a broken watcher
 *       degrades silently rather than failing loudly. Hence there is nothing in {@code accept} that
 *       can throw on a hostile value: field copies and null checks only, with every sanitising rule
 *       left to {@code PilotInputMapper}, which #17 documented as never throwing on a sample's
 *       account.
 *   <li><b>The handler lists are {@code static} and JVM-global</b> (`:13-14`), not per-plugin, so a
 *       watcher left behind by a hot reload keeps running alongside its replacement. See
 *       {@link #register} for the matched pair that prevents it.
 *   <li><b>{@code deregisterInbound} throws if the handler was never registered</b> (`:90-96`), and
 *       the key is the {@link PacketFilter} the {@code register*} call <em>returned</em> — the wrapper
 *       it built around the watcher, not the watcher. So that wrapper is what {@link #register}
 *       returns and {@link #deregister} consumes.
 * </ul>
 */
public final class ClientMovementWatcher implements PlayerPacketWatcher {

    @Nonnull
    private final PilotInputBuffer buffer;

    /**
     * The netty-side half of #49's instrument. Written here, read by {@code /fpv input status}.
     *
     * <p>{@code AtomicLong} rather than plain counters because {@link #accept} runs on netty threads
     * and the command that reads them runs on the world thread. The cost is irrelevant next to
     * deserialising a packet; the alternative — an unsynchronised long — would report numbers a
     * human would then reason from, which is worse than reporting nothing.
     */
    @Nonnull
    private final AtomicLong inboundPackets = new AtomicLong();
    @Nonnull
    private final AtomicLong movementPackets = new AtomicLong();
    @Nonnull
    private final AtomicLong watcherErrors = new AtomicLong();

    public ClientMovementWatcher(@Nonnull PilotInputBuffer buffer) {
        this.buffer = buffer;
    }

    /**
     * Subscribes to the inbound packet stream and answers the handle needed to unsubscribe.
     *
     * <p>Call this from the plugin's {@code start()} and pass the result to {@link #deregister} from
     * {@code shutdown()}. That pairing is exact, unlike registering in {@code setup()}:
     * {@code PluginBase.start0} only calls {@code start()} when {@code setup()} succeeded and only
     * reaches {@code ENABLED} when {@code start()} returned (`server/core/plugin/PluginBase.java:259-274`),
     * and {@code PluginManager.shutdown} only calls {@code shutdown0} for an {@code ENABLED} plugin
     * (`server/core/plugin/PluginManager.java:398`). Register in {@code setup()} and any later setup
     * failure leaks this watcher into a static list for the life of the JVM.
     */
    @Nonnull
    public PacketFilter register() {
        return PacketAdapters.registerInbound(this);
    }

    /** Unsubscribes. Tolerates {@code null} so a failed startup does not throw on the way out. */
    public static void deregister(@Nullable PacketFilter handle) {
        if (handle != null) {
            PacketAdapters.deregisterInbound(handle);
        }
    }

    @Override
    public void accept(PlayerRef playerRef, Packet packet) {
        accept(playerRef == null ? null : playerRef.getUuid(), packet);
    }

    /**
     * All of {@link #accept} bar the single {@code PlayerRef} field read, so that everything with
     * behaviour in it is reachable from a plain JVM test.
     *
     * <p>The harness cannot construct a {@code PlayerRef} — the constraint that made #19 introduce
     * its {@code PilotSink} seam, and the reason #47's key-mismatch hypothesis survived four flights
     * unfalsified. Splitting here means the counters, the packet-type filter and the error path are
     * all under test; only "which UUID a connection reports" is not, and that is now pinned by
     * reading instead ({@code Universe.addPlayer} builds {@code PlayerRef} and
     * {@code UUIDComponent} from the same {@code auth.getUuid()}).
     */
    void accept(@Nullable UUID pilotId, @Nullable Packet packet) {
        this.inboundPackets.incrementAndGet();
        try {
            if (pilotId == null || !(packet instanceof ClientMovement movement)) {
                return;
            }
            this.movementPackets.incrementAndGet();
            // Converted to an immutable sample immediately: the packet object belongs to the netty
            // pipeline and nothing of ours should outlive this call holding a reference to it.
            this.buffer.offer(pilotId, ClientMovementAdapter.sample(movement));
        } catch (Throwable t) {
            // Counted and rethrown, not swallowed. PacketAdapters.handle logs it at SEVERE
            // (`PacketAdapters.java:104-106`) and carries on, so behaviour is unchanged; what
            // changes is that a watcher throwing on every packet is no longer indistinguishable
            // from one that never ran. #49 exists because that distinction was unavailable.
            this.watcherErrors.incrementAndGet();
            throw t;
        }
    }

    /**
     * Inbound packets of <em>any</em> type this watcher has seen.
     *
     * <p>Zero means the watcher is not in the pipeline at all — a registration or hot-reload fault,
     * not an input-mapping one. Non-zero with {@link #movementPackets()} at zero means the pipeline
     * is fine and the client is genuinely not sending packet 108.
     */
    public long inboundPackets() {
        return this.inboundPackets.get();
    }

    /** {@code ClientMovement} packets seen, i.e. how often the input path actually ran. */
    public long movementPackets() {
        return this.movementPackets.get();
    }

    /** Throws out of {@link #accept}. Any non-zero value here invalidates the counts below it. */
    public long watcherErrors() {
        return this.watcherErrors.get();
    }
}
