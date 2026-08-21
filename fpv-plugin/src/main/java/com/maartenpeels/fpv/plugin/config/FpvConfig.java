package com.maartenpeels.fpv.plugin.config;

import com.hypixel.hytale.codec.Codec;
import com.hypixel.hytale.codec.KeyedCodec;
import com.hypixel.hytale.codec.builder.BuilderCodec;
import com.maartenpeels.fpv.plugin.camera.DroneCameraTuning;
import com.maartenpeels.fpv.plugin.camera.DroneCameraView;

import javax.annotation.Nonnull;

/**
 * Server-wide plugin configuration.
 *
 * <p>Holds the two knobs CLAUDE.md decision 3 requires to stay cheap: the substep count the
 * integrator runs per tick, and the tick rate the drone world runs at. If flight feels floaty at
 * 30 TPS, raising {@code WorldTps} is the documented escape hatch, and it has to remain a config
 * change rather than a rewrite.
 *
 * <p>Plus the FPV camera settings (#19). Those are here for a sharper reason: #19 is the one phase-0
 * feature that <em>no test can verify</em>, because the server keeps no record of what it told a
 * client the camera should follow. Getting it right is a human flying and changing knobs, so every
 * knob that might be the difference between right and wrong has to be reachable without a rebuild.
 * See {@link DroneCameraTuning}.
 *
 * <p>Per-pilot tuning (rates, PID, profiles) does <em>not</em> belong here — that is
 * {@code PilotProfile} in {@code :fpv-core}, persisted per player.
 */
public class FpvConfig {

    /** Hytale's own default; see {@code WorldTpsResetCommand} in the decompiled server. */
    public static final int DEFAULT_WORLD_TPS = 30;

    /** 8 substeps at 30 TPS is ~4 ms per integration step. */
    public static final int DEFAULT_PHYSICS_SUBSTEPS = 8;

    public static final BuilderCodec<FpvConfig> CODEC =
            BuilderCodec.builder(FpvConfig.class, FpvConfig::new)
                    .append(
                            new KeyedCodec<>("PhysicsSubsteps", Codec.INTEGER),
                            (config, value, extraInfo) -> config.physicsSubsteps = value,
                            (config, extraInfo) -> config.physicsSubsteps)
                    .add()
                    .append(
                            new KeyedCodec<>("WorldTps", Codec.INTEGER),
                            (config, value, extraInfo) -> config.worldTps = value,
                            (config, extraInfo) -> config.worldTps)
                    .add()
                    .append(
                            new KeyedCodec<>("CameraView", Codec.STRING),
                            (config, value, extraInfo) -> config.cameraView = value,
                            (config, extraInfo) -> config.cameraView)
                    .add()
                    .append(
                            new KeyedCodec<>("CameraPositionLerpSpeed", Codec.FLOAT),
                            (config, value, extraInfo) -> config.cameraPositionLerpSpeed = value,
                            (config, extraInfo) -> config.cameraPositionLerpSpeed)
                    .add()
                    .append(
                            new KeyedCodec<>("CameraRotationLerpSpeed", Codec.FLOAT),
                            (config, value, extraInfo) -> config.cameraRotationLerpSpeed = value,
                            (config, extraInfo) -> config.cameraRotationLerpSpeed)
                    .add()
                    .append(
                            new KeyedCodec<>("CameraLocked", Codec.BOOLEAN),
                            (config, value, extraInfo) -> config.cameraLocked = value,
                            (config, extraInfo) -> config.cameraLocked)
                    .add()
                    .append(
                            new KeyedCodec<>("CameraUptiltDegrees", Codec.FLOAT),
                            (config, value, extraInfo) -> config.cameraUptiltDegrees = value,
                            (config, extraInfo) -> config.cameraUptiltDegrees)
                    .add()
                    .build();

    private int physicsSubsteps = DEFAULT_PHYSICS_SUBSTEPS;
    private int worldTps = DEFAULT_WORLD_TPS;

    private String cameraView = DroneCameraTuning.DEFAULT.view().name();
    private float cameraPositionLerpSpeed = DroneCameraTuning.DEFAULT.positionLerpSpeed();
    private float cameraRotationLerpSpeed = DroneCameraTuning.DEFAULT.rotationLerpSpeed();
    private boolean cameraLocked = DroneCameraTuning.DEFAULT.locked();
    private float cameraUptiltDegrees;

    private FpvConfig() {}

    /** Fixed integration substeps per server tick. */
    public int getPhysicsSubsteps() {
        return this.physicsSubsteps;
    }

    /** Tick rate for the world drones fly in. {@code World.setTps} accepts up to 2048. */
    public int getWorldTps() {
        return this.worldTps;
    }

    /** Seconds of simulated time per integration substep at the configured tick rate. */
    public float substepSeconds() {
        return 1.0f / (this.worldTps * this.physicsSubsteps);
    }

    /**
     * The camera settings the plugin starts with, as a validated value.
     *
     * <p><strong>Degrees in the file, radians on the wire.</strong> {@code CameraUptiltDegrees} is
     * authored in degrees because a human edits it, and converted exactly once, here.
     * {@code Direction} is radians throughout the protocol —
     * {@code PlayerCameraTopdownCommand.java:50} passes {@code -Math.PI / 2} to look straight down —
     * and a degrees value reaching the wire is roughly five rotations that reads as "the client
     * ignores this field". That misreading nearly derailed #28.
     *
     * <p>Falls back to {@link DroneCameraTuning#DEFAULT} rather than throwing when the file holds
     * something invalid. A camera preference must not be able to stop the plugin loading, and every
     * one of these values is visible the moment someone flies.
     */
    @Nonnull
    public DroneCameraTuning cameraTuning() {
        DroneCameraView view = DroneCameraView.parse(this.cameraView, DroneCameraTuning.DEFAULT.view());
        try {
            return new DroneCameraTuning(
                    view,
                    this.cameraPositionLerpSpeed,
                    this.cameraRotationLerpSpeed,
                    this.cameraLocked,
                    DroneCameraTuning.DEFAULT_EYE_HEIGHT,
                    (float) Math.toRadians(this.cameraUptiltDegrees));
        } catch (IllegalArgumentException e) {
            return DroneCameraTuning.DEFAULT.withView(view);
        }
    }
}
