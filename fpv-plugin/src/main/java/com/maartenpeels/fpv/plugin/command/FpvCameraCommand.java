package com.maartenpeels.fpv.plugin.command;

import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import com.hypixel.hytale.protocol.io.PacketStatsRecorder;
import com.hypixel.hytale.server.core.Message;
import com.hypixel.hytale.server.core.command.system.CommandContext;
import com.hypixel.hytale.server.core.command.system.arguments.system.OptionalArg;
import com.hypixel.hytale.server.core.command.system.arguments.types.ArgTypes;
import com.hypixel.hytale.server.core.command.system.basecommands.AbstractCommandCollection;
import com.hypixel.hytale.server.core.command.system.basecommands.AbstractTargetPlayerCommand;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.permissions.provider.HytalePermissionsProvider;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.world.World;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.FPVDrone;
import com.maartenpeels.fpv.plugin.camera.DroneCamera;
import com.maartenpeels.fpv.plugin.camera.DroneCameraTuning;
import com.maartenpeels.fpv.plugin.camera.DroneCameraView;
import com.maartenpeels.fpv.plugin.drone.DroneModel;
import com.maartenpeels.fpv.plugin.drone.FlightSession;
import com.maartenpeels.fpv.plugin.drone.FlightSessions;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * {@code /fpv camera …} — the instrument for the one phase-0 feature no test can check.
 *
 * <p><strong>Why this is part of #19 rather than gold-plating.</strong> The server keeps no record
 * of what it told a client the camera should follow, so whether the horizon actually tilts can only
 * be established by a human looking at one. Meanwhile #22 (`/fpv launch`, `/fpv land`) does not
 * exist yet and #24 (the flight tick) does not exist yet — so without these subcommands there is
 * no way to start a session and nothing to bank the drone, and the ticket would ship completely
 * unverified. Making the human's loop cheap is part of the work.
 *
 * <p>Four things it makes possible without a rebuild and without a flight model:
 *
 * <ul>
 *   <li>{@code on} / {@code off} — start and end a session. #22 should absorb or delete these.
 *   <li>{@code attitude --roll 30} — write an attitude straight into the drone, in degrees. This is
 *       what makes the horizon tilt with no physics at all, and it closes the exact gap #28 left:
 *       roll on a <em>real attached drone entity</em> rather than on the local player.
 *   <li>{@code set --view driven} — bisect the two camera modes, which is the only way to tell
 *       "the client ignores roll" apart from "the client ignores {@code AttachedToType.EntityId}".
 *   <li>{@code status} — read the packet-280 send counter, which separates "we never sent it" from
 *       "we sent it and the client ignored it".
 * </ul>
 *
 * <p><strong>Optional arguments are named, not positional</strong> — the trap that cost #28 half
 * its answers. {@code AbstractCommand.processOptionalArguments} resolves each optional arg by name
 * out of {@code parserContext.getOptionalArgs()}, and {@code CommandTreeBuilder:72} builds the
 * prefix as {@code "--" + name}. So it is {@code /fpv camera attitude --roll 30}, never
 * {@code /fpv camera attitude 30}.
 */
public class FpvCameraCommand extends AbstractCommandCollection {

    public FpvCameraCommand(@Nonnull FPVDrone plugin) {
        super("camera", "Inspect and tune the FPV camera (diagnostics for #19)");
        this.setPermissionGroups(HytalePermissionsProvider.GROUP_ADVENTURER);
        this.addSubCommand(new On(plugin));
        this.addSubCommand(new Off(plugin));
        this.addSubCommand(new Status(plugin));
        this.addSubCommand(new Set(plugin));
        this.addSubCommand(new Attitude(plugin));
    }

    /**
     * Shared plumbing: every subcommand needs the plugin's services and runs against one player.
     *
     * <p>{@code AbstractTargetPlayerCommand} is the right base because its
     * {@code execute} body runs on the <strong>world thread, outside any system tick</strong> —
     * {@code runAsync(context, runnable, targetWorld)} schedules onto the {@code World}, which is an
     * {@code Executor} draining its queue in {@code consumeTaskQueue}. That is exactly the contract
     * {@code FlightSessions} documents, since {@code Store.addEntity} asserts both the thread and
     * that no write is in progress.
     */
    private abstract static class CameraSubCommand extends AbstractTargetPlayerCommand {

        @Nonnull
        final FPVDrone plugin;

        CameraSubCommand(@Nonnull FPVDrone plugin, @Nonnull String name, @Nonnull String description) {
            super(name, description);
            this.setPermissionGroups(HytalePermissionsProvider.GROUP_ADVENTURER);
            this.plugin = plugin;
        }

        /** The camera service, or {@code null} having already told the sender why not. */
        @Nullable
        DroneCamera camera(@Nonnull CommandContext context) {
            DroneCamera camera = this.plugin.getDroneCamera();
            if (camera == null) {
                context.sendMessage(Message.raw("FPV camera is not initialised — plugin setup did not run."));
            }
            return camera;
        }

        /** The session service, or {@code null} having already told the sender why not. */
        @Nullable
        FlightSessions sessions(@Nonnull CommandContext context) {
            FlightSessions sessions = this.plugin.getFlightSessions();
            if (sessions == null) {
                context.sendMessage(Message.raw("FPV sessions are not initialised — plugin setup did not run."));
            }
            return sessions;
        }

        /** The pilot's drone, or {@code null} having already told the sender they are not flying. */
        @Nullable
        Ref<EntityStore> droneOf(
                @Nonnull CommandContext context,
                @Nonnull Store<EntityStore> store,
                @Nonnull Ref<EntityStore> pilot) {

            FlightSessions sessions = sessions(context);
            if (sessions == null) {
                return null;
            }
            Ref<EntityStore> drone = sessions.droneOf(store, pilot);
            if (drone == null) {
                context.sendMessage(Message.raw("Not flying. Run /fpv camera on first."));
            }
            return drone;
        }
    }

    /**
     * Starts a diagnostic session. The camera attaches by itself — {@code AttachOnSession} fires on
     * the {@link FlightSession} component appearing, so this command never mentions a camera.
     */
    private static final class On extends CameraSubCommand {

        On(@Nonnull FPVDrone plugin) {
            super(plugin, "on", "Spawn a drone at your feet and attach your camera to it");
        }

        @Override
        protected void execute(
                @Nonnull CommandContext context,
                @Nullable Ref<EntityStore> sourceRef,
                @Nonnull Ref<EntityStore> ref,
                @Nonnull PlayerRef playerRef,
                @Nonnull World world,
                @Nonnull Store<EntityStore> store) {

            FlightSessions sessions = sessions(context);
            if (sessions == null) {
                return;
            }

            TransformComponent transform =
                    store.getComponent(ref, TransformComponent.getComponentType());
            if (transform == null) {
                context.sendMessage(Message.raw("You have no transform — cannot place a drone."));
                return;
            }

            // Spawn facing the way the pilot is facing, so "forward" means what they expect on the
            // very first input. Roll starts at zero: a nonzero start would be indistinguishable
            // from the camera misreading the roll field.
            Rotation3f rotation = new Rotation3f(0.0f, transform.getRotation().yaw(), 0.0f);

            Ref<EntityStore> drone = sessions.launch(
                    store, ref, transform.getPosition(), rotation, DroneModel.resolve());
            if (drone == null) {
                context.sendMessage(Message.raw("Already flying, or the spawn was rejected. Try /fpv camera off."));
                return;
            }

            NetworkId networkId = store.getComponent(drone, NetworkId.getComponentType());
            context.sendMessage(Message.raw(String.format(
                    "Drone spawned (networkId=%s). Camera should now be in it. "
                            + "Try: /fpv camera attitude --roll 30",
                    networkId == null ? "?" : Integer.toString(networkId.getId()))));
        }
    }

    /** Ends the session. Detach is again automatic, on the session component being removed. */
    private static final class Off extends CameraSubCommand {

        Off(@Nonnull FPVDrone plugin) {
            super(plugin, "off", "Remove your drone and return your camera to normal");
        }

        @Override
        protected void execute(
                @Nonnull CommandContext context,
                @Nullable Ref<EntityStore> sourceRef,
                @Nonnull Ref<EntityStore> ref,
                @Nonnull PlayerRef playerRef,
                @Nonnull World world,
                @Nonnull Store<EntityStore> store) {

            FlightSessions sessions = sessions(context);
            if (sessions == null) {
                return;
            }

            if (sessions.land(store, ref)) {
                context.sendMessage(Message.raw("Landed. Your view should be back to normal."));
                return;
            }

            // Not flying, so no session removal will fire a detach -- but the pilot may still be
            // holding a camera from a previous run that ended badly. Sending the release is
            // harmless and is the difference between "stuck in a weird view" and "fixed".
            DroneCamera camera = camera(context);
            if (camera != null && camera.detach(store, ref)) {
                context.sendMessage(Message.raw("Was not flying; sent a camera release anyway."));
            } else {
                context.sendMessage(Message.raw("Was not flying."));
            }
        }
    }

    /**
     * Reports what we think we sent, and how many times we actually sent it.
     *
     * <p>The send counter is the whole point. {@code PacketStatsRecorder} is created unconditionally
     * for every connection (`server/core/io/netty/HytaleChannelInitializer.java:93-94`) and counted
     * in the encoder, so {@code getEntry(280).getSentCount()} is ground truth about the wire. When
     * the horizon does not tilt, this is the first thing to look at: a count of zero is our bug, and
     * a count that climbs is the client's.
     */
    private static final class Status extends CameraSubCommand {

        Status(@Nonnull FPVDrone plugin) {
            super(plugin, "status", "Show camera tuning and the number of camera packets sent");
        }

        @Override
        protected void execute(
                @Nonnull CommandContext context,
                @Nullable Ref<EntityStore> sourceRef,
                @Nonnull Ref<EntityStore> ref,
                @Nonnull PlayerRef playerRef,
                @Nonnull World world,
                @Nonnull Store<EntityStore> store) {

            DroneCamera camera = camera(context);
            if (camera == null) {
                return;
            }
            DroneCameraTuning tuning = camera.getTuning();

            FlightSessions sessions = this.plugin.getFlightSessions();
            Ref<EntityStore> drone = sessions == null ? null : sessions.droneOf(store, ref);
            NetworkId networkId =
                    drone == null ? null : store.getComponent(drone, NetworkId.getComponentType());

            PacketStatsRecorder recorder = playerRef.getPacketHandler().getPacketStatsRecorder();
            String sent = recorder == null
                    ? "unavailable"
                    : Integer.toString(recorder.getEntry(SetServerCamera.PACKET_ID).getSentCount());

            context.sendMessage(Message.raw(String.format(
                    "FPV camera: view=%s rotationType=%s locked=%s posLerp=%.3f rotLerp=%.3f "
                            + "eyeHeight=%.3f uptilt=%.2f deg%n"
                            + "  flying=%s droneNetworkId=%s%n"
                            + "  SetServerCamera (packet %d) sent to you: %s",
                    tuning.view(),
                    tuning.view().rotationType(),
                    tuning.locked(),
                    tuning.positionLerpSpeed(),
                    tuning.rotationLerpSpeed(),
                    tuning.eyeHeight(),
                    Math.toDegrees(tuning.cameraUptiltRadians()),
                    drone != null,
                    networkId == null ? "-" : Integer.toString(networkId.getId()),
                    SetServerCamera.PACKET_ID,
                    sent)));
        }
    }

    /**
     * Changes the camera tuning and re-attaches so the change is visible immediately.
     *
     * <p>Server-wide, not per pilot: this is a diagnostic for one person answering one question, and
     * a per-pilot override would be state to reconcile for no benefit while #19 is still unproven.
     */
    private static final class Set extends CameraSubCommand {

        @Nonnull
        private final OptionalArg<String> viewArg =
                this.withOptionalArg("view", "tracked (AttachedToPlusOffset) or driven (Custom)", ArgTypes.STRING);
        @Nonnull
        private final OptionalArg<Float> posLerpArg =
                this.withOptionalArg("poslerp", "Client position smoothing, >0 (Hytale uses 0.2)", ArgTypes.FLOAT);
        @Nonnull
        private final OptionalArg<Float> rotLerpArg =
                this.withOptionalArg("rotlerp", "Client rotation smoothing, >0 (Hytale uses 0.2)", ArgTypes.FLOAT);
        @Nonnull
        private final OptionalArg<Boolean> lockedArg =
                this.withOptionalArg("locked", "The packet's isLocked flag", ArgTypes.BOOLEAN);
        @Nonnull
        private final OptionalArg<Float> uptiltArg =
                this.withOptionalArg("uptilt", "Camera tilt in DEGREES, positive nose-up", ArgTypes.FLOAT);

        Set(@Nonnull FPVDrone plugin) {
            super(plugin, "set", "Change camera view mode, smoothing, lock or tilt, and re-attach");
        }

        @Override
        protected void execute(
                @Nonnull CommandContext context,
                @Nullable Ref<EntityStore> sourceRef,
                @Nonnull Ref<EntityStore> ref,
                @Nonnull PlayerRef playerRef,
                @Nonnull World world,
                @Nonnull Store<EntityStore> store) {

            DroneCamera camera = camera(context);
            if (camera == null) {
                return;
            }

            DroneCameraTuning tuning = camera.getTuning();

            if (this.viewArg.provided(context)) {
                String raw = this.viewArg.get(context);
                // parseOrNull, not parse: falling back on a typo would silently keep the old mode,
                // and this command's entire job is telling two modes apart.
                DroneCameraView parsed = DroneCameraView.parseOrNull(raw);
                if (parsed == null) {
                    context.sendMessage(Message.raw("Unknown view '" + raw + "'. Use: tracked | driven"));
                    return;
                }
                tuning = tuning.withView(parsed);
            }

            float posLerp = this.posLerpArg.provided(context)
                    ? this.posLerpArg.get(context)
                    : tuning.positionLerpSpeed();
            float rotLerp = this.rotLerpArg.provided(context)
                    ? this.rotLerpArg.get(context)
                    : tuning.rotationLerpSpeed();
            tuning = tuning.withLerpSpeeds(posLerp, rotLerp);

            if (this.lockedArg.provided(context)) {
                tuning = tuning.withLocked(this.lockedArg.get(context));
            }

            if (this.uptiltArg.provided(context)) {
                // Degrees in, radians out, converted exactly once. Direction is radians throughout
                // the protocol; a degrees value on the wire is ~5 rotations and reads as garbage.
                tuning = tuning.withCameraUptiltRadians(
                        (float) Math.toRadians(this.uptiltArg.get(context)));
            }

            DroneCameraTuning validated;
            try {
                // Re-run the record's own validation by round-tripping through the canonical
                // constructor, so a NaN or non-positive lerp is rejected here with a message rather
                // than shipped to the client as a camera that never moves.
                validated = new DroneCameraTuning(
                        tuning.view(),
                        tuning.positionLerpSpeed(),
                        tuning.rotationLerpSpeed(),
                        tuning.locked(),
                        tuning.eyeHeight(),
                        tuning.cameraUptiltRadians());
            } catch (IllegalArgumentException e) {
                context.sendMessage(Message.raw("Rejected: " + e.getMessage()));
                return;
            }

            camera.setTuning(validated);

            boolean reattached = camera.attachToSessionDrone(store, ref);
            context.sendMessage(Message.raw(String.format(
                    "view=%s locked=%s posLerp=%.3f rotLerp=%.3f uptilt=%.2f deg — %s",
                    validated.view(),
                    validated.locked(),
                    validated.positionLerpSpeed(),
                    validated.rotationLerpSpeed(),
                    Math.toDegrees(validated.cameraUptiltRadians()),
                    reattached ? "re-attached" : "not flying, will apply on next /fpv camera on")));
        }
    }

    /**
     * Writes an attitude straight into the drone, in degrees.
     *
     * <p>The key instrument of this ticket: with no flight model (#24) nothing rolls the drone, so
     * without this there is nothing to look at. It closes the gap #28 explicitly left — that spike
     * rolled the <em>local player's</em> camera, which says nothing about roll surviving
     * {@code attachedToType = EntityId} plus {@code skipCharacterPhysics} on a real entity.
     *
     * <p>It sets <strong>both</strong> {@code TransformComponent} and {@code HeadRotation}. Those
     * become {@code ModelTransform.bodyOrientation} and {@code .lookOrientation} respectively
     * (`server/core/modules/entity/system/TransformSystems.java:75-85`), and which of the two a
     * client uses for {@code RotationType.AttachedToPlusOffset} is unknowable from the server
     * sources — there is no client tree in the decompiled cache. Setting both removes that
     * variable. <strong>#24 must do the same</strong> or {@code TRACKED} will read as broken.
     */
    private static final class Attitude extends CameraSubCommand {

        @Nonnull
        private final OptionalArg<Float> rollArg =
                this.withOptionalArg("roll", "Roll in DEGREES (right wing down positive)", ArgTypes.FLOAT);
        @Nonnull
        private final OptionalArg<Float> pitchArg =
                this.withOptionalArg("pitch", "Pitch in DEGREES, Hytale convention: positive nose-UP", ArgTypes.FLOAT);
        @Nonnull
        private final OptionalArg<Float> yawArg =
                this.withOptionalArg("yaw", "Yaw in DEGREES", ArgTypes.FLOAT);

        Attitude(@Nonnull FPVDrone plugin) {
            super(plugin, "attitude", "Force your drone's attitude in degrees, to see whether the camera follows");
        }

        @Override
        protected void execute(
                @Nonnull CommandContext context,
                @Nullable Ref<EntityStore> sourceRef,
                @Nonnull Ref<EntityStore> ref,
                @Nonnull PlayerRef playerRef,
                @Nonnull World world,
                @Nonnull Store<EntityStore> store) {

            Ref<EntityStore> drone = droneOf(context, store, ref);
            if (drone == null) {
                return;
            }

            TransformComponent transform =
                    store.getComponent(drone, TransformComponent.getComponentType());
            if (transform == null) {
                context.sendMessage(Message.raw("Drone has no transform."));
                return;
            }

            Rotation3f current = transform.getRotation();
            float rollDeg = this.rollArg.provided(context)
                    ? this.rollArg.get(context)
                    : (float) Math.toDegrees(current.roll());
            float pitchDeg = this.pitchArg.provided(context)
                    ? this.pitchArg.get(context)
                    : (float) Math.toDegrees(current.pitch());
            float yawDeg = this.yawArg.provided(context)
                    ? this.yawArg.get(context)
                    : (float) Math.toDegrees(current.yaw());

            if (!Float.isFinite(rollDeg) || !Float.isFinite(pitchDeg) || !Float.isFinite(yawDeg)) {
                // A NaN angle renders as no tilt at all, which reads as "the client ignores roll" --
                // the precise false negative this whole instrument exists to avoid.
                context.sendMessage(Message.raw("Angles must be finite numbers."));
                return;
            }

            // Rotation3f is (pitch, yaw, roll); Direction is (yaw, pitch, roll). Transposed on the
            // first two. Radians on both sides -- degrees are converted here and nowhere else.
            Rotation3f rotation = new Rotation3f(
                    (float) Math.toRadians(pitchDeg),
                    (float) Math.toRadians(yawDeg),
                    (float) Math.toRadians(rollDeg));

            transform.setRotation(rotation);
            HeadRotation head = store.getComponent(drone, HeadRotation.getComponentType());
            if (head != null) {
                head.setRotation(rotation);
            }

            context.sendMessage(Message.raw(String.format(
                    "Drone attitude set: roll=%.1f pitch=%.1f yaw=%.1f deg "
                            + "(%.4f / %.4f / %.4f rad). View mode is %s.",
                    rollDeg,
                    pitchDeg,
                    yawDeg,
                    Math.toRadians(rollDeg),
                    Math.toRadians(pitchDeg),
                    Math.toRadians(yawDeg),
                    this.plugin.getDroneCamera() == null
                            ? "?"
                            : this.plugin.getDroneCamera().getTuning().view().toString())));
        }
    }
}
