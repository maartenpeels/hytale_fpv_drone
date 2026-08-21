package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.server.core.asset.type.model.config.Model;
import com.hypixel.hytale.server.core.asset.type.model.config.ModelAsset;

import javax.annotation.Nonnull;

/**
 * Resolves the drone's appearance from the asset pack.
 *
 * <p>The only code in the feature that needs a loaded asset registry, and therefore the only code
 * the harness cannot reach: {@code ModelAsset.getAssetMap()} goes through {@code AssetRegistry},
 * which needs {@code Assets.zip}. Everything else takes the resolved {@link Model} as a parameter.
 */
public final class DroneModel {

    /**
     * Asset id of the drone model.
     *
     * <p>The id is the <em>filename</em> of the asset JSON, not a field inside it:
     * {@code AssetBuilderCodec.decodeAndInheritJsonAsset} overwrites any {@code Id} in the body
     * with the file path key (`assetstore/codec/AssetBuilderCodec.java:67-78`). So this must match
     * {@code fpv-plugin/src/main/resources/Server/Models/FpvDrone.json}.
     */
    public static final String ASSET_ID = "FpvDrone";

    private DroneModel() {}

    /**
     * The drone model, falling back to Hytale's debug gizmo if the pack asset is missing.
     *
     * <p>The fallback is the idiom at {@code SpawnMinecartInteraction.java:103}, and it is a
     * deliberate safety property: a typo or a missing pack gives a visible three-axis marker
     * rather than an invisible drone, so "I can't see anything" is never ambiguous between a
     * broken asset and broken physics. {@code ModelAsset.DEBUG} is a real static with a unit
     * bounding box and unit scale (`ModelAsset.java:298-308`), injected into the registry via
     * {@code preLoadAssets} (`AssetRegistryLoader.java:650`).
     *
     * <p>Unit scale rather than {@code createRandomScaleModel}, which would pick a random size
     * between the asset's {@code MinScale} and {@code MaxScale} — a randomly-sized drone changes
     * how it flies.
     */
    @Nonnull
    public static Model resolve() {
        ModelAsset asset = ModelAsset.getAssetMap().getAsset(ASSET_ID);
        return Model.createUnitScaleModel(asset != null ? asset : ModelAsset.DEBUG);
    }
}
