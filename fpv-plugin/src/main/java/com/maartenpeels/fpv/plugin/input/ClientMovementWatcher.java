package com.maartenpeels.fpv.plugin.input;

import com.hypixel.hytale.protocol.Packet;
import com.hypixel.hytale.server.core.io.adapter.PacketAdapters;
import com.hypixel.hytale.server.core.io.adapter.PacketFilter;
import com.hypixel.hytale.server.core.io.adapter.PlayerPacketWatcher;
import com.hypixel.hytale.protocol.packets.player.ClientMovement;
import com.hypixel.hytale.server.core.universe.PlayerRef;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

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
        if (playerRef == null || !(packet instanceof ClientMovement movement)) {
            return;
        }
        // Converted to an immutable sample immediately: the packet object belongs to the netty
        // pipeline and nothing of ours should outlive this call holding a reference to it.
        this.buffer.offer(playerRef.getUuid(), ClientMovementAdapter.sample(movement));
    }
}
