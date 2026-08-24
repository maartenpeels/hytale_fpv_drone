package com.maartenpeels.fpv.plugin.command;

import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.protocol.io.PacketStatsRecorder;
import com.hypixel.hytale.protocol.packets.player.ClientMovement;
import com.hypixel.hytale.server.core.Message;
import com.hypixel.hytale.server.core.command.system.CommandContext;
import com.hypixel.hytale.server.core.command.system.basecommands.AbstractCommandCollection;
import com.hypixel.hytale.server.core.command.system.basecommands.AbstractTargetPlayerCommand;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.permissions.provider.HytalePermissionsProvider;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.world.World;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.FPVDrone;
import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.control.PilotInputSample;
import com.maartenpeels.fpv.plugin.input.ClientMovementWatcher;
import com.maartenpeels.fpv.plugin.input.PilotInputBuffer;
import com.maartenpeels.fpv.plugin.input.PilotInputSlot;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Set;
import java.util.UUID;

/**
 * {@code /fpv input …} — the inbound half of the flight-test instrument.
 *
 * <p><strong>Why this exists.</strong> Four flight sessions produced symptoms rather than diagnoses
 * because nothing reported the inbound side. {@code /fpv camera status} counts <em>outbound</em>
 * packet 280s; there was no way to see whether a {@code ClientMovement} arrived, whether it was
 * routed to a slot, or what stick positions it became. That gap is what made #47 a four-hypothesis
 * guess.
 *
 * <p>Two of those hypotheses have since been closed without this command — the camera lock, by
 * flying with {@code --locked false}, and the UUID key mismatch, by reading
 * {@code Universe.addPlayer}, which builds {@code PlayerRef} and {@code UUIDComponent} from the same
 * {@code auth.getUuid()}. What remains cannot be closed from outside the process, because #45 made
 * every failure mode look identical: a missing slot, a slot with no packets, and a slot fed empty
 * packets all end at {@code PilotInputMapper.hovering()}, i.e. a drone sitting perfectly still.
 *
 * <p>So {@link Status} prints the whole chain and then <em>names</em> the failure mode. The naming is
 * the deliverable: a table of eight numbers still needs interpreting mid-flight, which is exactly
 * when a pilot is least able to do it.
 */
public class FpvInputCommand extends AbstractCommandCollection {

    public FpvInputCommand(@Nonnull FPVDrone plugin) {
        super("input", "Inspect the inbound pilot-input path (diagnostics for #47)");
        this.setPermissionGroups(HytalePermissionsProvider.GROUP_ADVENTURER);
        this.addSubCommand(new Status(plugin));
    }

    /**
     * Reports every stage of the input path for the running player, and the verdict.
     *
     * <p>Runs on the world thread, outside any system tick — {@code AbstractTargetPlayerCommand}
     * schedules onto the {@code World} — so reading {@link PilotInputSlot}'s world-thread state is
     * legitimate here rather than a race.
     */
    private static final class Status extends AbstractTargetPlayerCommand {

        @Nonnull
        private final FPVDrone plugin;

        Status(@Nonnull FPVDrone plugin) {
            super("status", "Show whether pilot input is arriving, routed, fresh and mapped");
            this.setPermissionGroups(HytalePermissionsProvider.GROUP_ADVENTURER);
            this.plugin = plugin;
        }

        @Override
        protected void execute(
                @Nonnull CommandContext context,
                @Nullable Ref<EntityStore> sourceRef,
                @Nonnull Ref<EntityStore> ref,
                @Nonnull PlayerRef playerRef,
                @Nonnull World world,
                @Nonnull Store<EntityStore> store) {

            ClientMovementWatcher watcher = this.plugin.getMovementWatcher();
            PilotInputBuffer buffer = this.plugin.getPilotInputs();

            UUID fromPlayerRef = playerRef.getUuid();
            UUIDComponent uuidComponent =
                    store.getComponent(ref, UUIDComponent.getComponentType());
            UUID fromComponent = uuidComponent == null ? null : uuidComponent.getUuid();

            // The count the client's own connection kept, independent of anything we wrote. If this
            // and the watcher's movement count disagree, the fault is between the pipeline and us.
            PacketStatsRecorder recorder = playerRef.getPacketHandler().getPacketStatsRecorder();
            long received = recorder == null
                    ? -1L
                    : recorder.getEntry(ClientMovement.PACKET_ID).getReceivedCount();

            PilotInputSlot slot = buffer.slotOf(fromPlayerRef);
            Set<UUID> openKeys = buffer.openKeys();

            StringBuilder out = new StringBuilder();
            out.append("FPV input path:\n");
            out.append(String.format(
                    "  ClientMovement (packet %d) received from you: %s%n",
                    ClientMovement.PACKET_ID,
                    received < 0 ? "unavailable" : Long.toString(received)));

            if (watcher == null) {
                out.append("  watcher: NOT REGISTERED — plugin start() did not run\n");
            } else {
                out.append(String.format(
                        "  watcher: inbound(any)=%d movement=%d errors=%d%n",
                        watcher.inboundPackets(),
                        watcher.movementPackets(),
                        watcher.watcherErrors()));
            }

            out.append(String.format(
                    "  offers: accepted=%d dropped(no slot)=%d%n",
                    buffer.offersAccepted(),
                    buffer.offersDropped()));
            out.append(String.format(
                    "  your uuid: PlayerRef=%s UUIDComponent=%s match=%s%n",
                    fromPlayerRef,
                    fromComponent,
                    fromComponent != null && fromComponent.equals(fromPlayerRef)));
            out.append(String.format(
                    "  open slots: %d %s%n", openKeys.size(), openKeys));

            if (slot == null) {
                out.append("  slot: NONE for your uuid — offers cannot land\n");
            } else {
                ControlInput sticks = slot.lastInput();
                PilotInputSample held = slot.held();
                boolean stale = slot.heldSeconds() > PilotInputSlot.MAX_HELD_SECONDS;
                out.append(String.format(
                        "  slot: offers=%d sinceFreshPacket=%.3fs (cutoff %.3fs) stale=%s%n",
                        slot.offers(),
                        slot.heldSeconds(),
                        PilotInputSlot.MAX_HELD_SECONDS,
                        stale));
                out.append(String.format(
                        "  last sample: wishX=%.4f wishZ=%.4f frameYaw=%.4f lookYaw=%.4f lookPitch=%.4f%n",
                        held.wishX(),
                        held.wishZ(),
                        held.wishFrameYaw(),
                        held.lookYaw(),
                        held.lookPitch()));
                out.append(sticks == null
                        ? "  sticks: never ticked — AdvanceFlight has not run for this drone\n"
                        : String.format(
                                "  sticks: throttle=%.4f roll=%.4f pitch=%.4f yaw=%.4f%n",
                                sticks.throttle(),
                                sticks.roll(),
                                sticks.pitch(),
                                sticks.yaw()));
            }

            out.append("  verdict: ").append(verdict(watcher, buffer, slot, received));
            context.sendMessage(Message.raw(out.toString()));
        }

        /**
         * Names the failure mode, in the order the path runs, so the first broken stage is the one
         * reported rather than the last observable symptom.
         *
         * <p>Ordering matters more than wording here: a missing slot and an idle client both end in
         * centred sticks, so a verdict computed from the sticks backwards would blame the mapper for
         * both.
         */
        @Nonnull
        private static String verdict(
                @Nullable ClientMovementWatcher watcher,
                @Nonnull PilotInputBuffer buffer,
                @Nullable PilotInputSlot slot,
                long received) {

            if (watcher == null) {
                return "watcher never registered — this is ours, upstream of everything else.";
            }
            if (watcher.watcherErrors() > 0) {
                return "the watcher is THROWING — every count below it is unreliable. "
                        + "PacketAdapters logs the cause at SEVERE; read the server log.";
            }
            if (watcher.inboundPackets() == 0) {
                return "no inbound packets of any type reached the watcher — it is not in the "
                        + "pipeline. Suspect a hot reload leaving a stale registration.";
            }
            if (watcher.movementPackets() == 0) {
                return received > 0
                        ? "the connection received packet 108 but the watcher never saw one — the "
                                + "fault is between the netty pipeline and us."
                        : "the client is not sending packet 108 at all. This is client-side: "
                                + "suspect the camera settings suppressing movement input, not our "
                                + "mapping.";
            }
            if (slot == null) {
                return buffer.offersDropped() > 0
                        ? "packets are arriving and being DROPPED for want of a slot. Either you are "
                                + "not flying, or TrackPilotInput opened the slot on a different key."
                        : "packets are arriving but you have no slot — are you flying?";
            }
            if (slot.offers() == 0) {
                return "you have a slot and packets are arriving, but none reached this slot — a key "
                        + "mismatch is the only way that happens.";
            }
            if (slot.lastInput() == null) {
                return "input is arriving and routed, but the drone has never been ticked — "
                        + "AdvanceFlight is registered but not running.";
            }
            if (slot.heldSeconds() > PilotInputSlot.MAX_HELD_SECONDS) {
                return "routed, but the last packet is older than the staleness cutoff, so the drone "
                        + "is flying centred sticks. Input stopped rather than never started.";
            }
            if (slot.lastInput().sticksCentred()) {
                return "the whole path works and the sticks are genuinely centred — the packets are "
                        + "arriving empty. Read the last-sample line: all-zero wish with NaN look "
                        + "means the client is sending movement packets with no movement in them.";
            }
            return "input is arriving, routed, fresh and mapped to non-centred sticks. The input "
                    + "path is healthy; if the drone is not responding the fault is downstream.";
        }
    }
}
