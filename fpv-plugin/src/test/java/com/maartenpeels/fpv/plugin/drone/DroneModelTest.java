package com.maartenpeels.fpv.plugin.drone;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import javax.annotation.Nonnull;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.List;

/**
 * Structural checks on the drone's asset pack entry.
 *
 * <p>No test can load a Hytale asset pack — {@code AssetRegistry} needs the ~GB
 * {@code Assets.zip} and {@code CommonAssetValidator} resolves paths through
 * {@code CommonAssetRegistry}, which only exists in a booted server. So the mesh itself is
 * unverified until someone flies it.
 *
 * <p>What <em>is</em> checkable is every way the asset can be wrong without anyone noticing until
 * runtime: a path that does not resolve, geometry in a directory the validator rejects, a missing
 * {@code HitBox}, an id that does not match the filename, or scale bounds that make the drone a
 * different size on every launch. Those are the realistic failures, and they are silent — a model
 * asset that fails validation just means an invisible drone, which looks exactly like broken
 * physics. Hence this file.
 */
class DroneModelTest {

    private static final String ASSET_PATH = "/Server/Models/" + DroneModel.ASSET_ID + ".json";

    /**
     * Roots {@code CommonAssetValidator.MODEL_CHARACTER} and {@code TEXTURE_CHARACTER} accept
     * (`server/core/asset/common/CommonAssetValidator.java:16,36`). Anything outside these fails
     * validation at load time with "must be within the root".
     */
    private static final List<String> ALLOWED_COMMON_ROOTS = List.of("Characters/", "NPC/", "Items/", "VFX/");

    @Nonnull
    private static JsonObject readJson(@Nonnull String resource) throws IOException {
        try (InputStream in = DroneModelTest.class.getResourceAsStream(resource)) {
            assertNotNull(in, resource + " must be packaged in the plugin jar");
            return JsonParser.parseReader(new InputStreamReader(in, StandardCharsets.UTF_8)).getAsJsonObject();
        }
    }

    private static boolean existsInPack(@Nonnull String commonRelativePath) throws IOException {
        try (InputStream in = DroneModelTest.class.getResourceAsStream("/Common/" + commonRelativePath)) {
            return in != null;
        }
    }

    @Nested
    class TheModelAsset {

        @Test
        void livesAtThePathItsAssetIdImplies() throws IOException {
            // The asset id is the filename, not a field: AssetBuilderCodec overwrites any "Id" in
            // the body with the file path key (`assetstore/codec/AssetBuilderCodec.java:67-78`).
            // So DroneModel.ASSET_ID and the filename are one fact, and this is the only thing
            // that keeps them from drifting apart.
            assertNotNull(readJson(ASSET_PATH));
        }

        @Test
        void declaresAHitBoxBecauseItIsTheOnlyRequiredKey() throws IOException {
            // ModelAsset's codec puts a nonNull validator on HitBox (`ModelAsset.java:175-182`),
            // and Model.createScaledModel dereferences it when scale != 1
            // (`Model.java:594`), so omitting it is a load failure or an NPE.
            JsonObject hitBox = readJson(ASSET_PATH).getAsJsonObject("HitBox");
            assertNotNull(hitBox, "HitBox is required");

            for (String corner : List.of("Min", "Max")) {
                JsonObject vec = hitBox.getAsJsonObject(corner);
                assertNotNull(vec, "HitBox." + corner);
                for (String axis : List.of("X", "Y", "Z")) {
                    assertTrue(vec.has(axis), "HitBox." + corner + "." + axis + " is required");
                }
            }
        }

        @Test
        void pinsScaleToOneSoTheDroneIsNotARandomSizeEachLaunch() throws IOException {
            // ModelAsset defaults MinScale/MaxScale to 0.95/1.05 (`ModelAsset.java:152-167`).
            // Leaving them defaulted would make mass, drag and reach vary per spawn.
            JsonObject asset = readJson(ASSET_PATH);
            assertEquals(1.0, asset.get("MinScale").getAsDouble(), 0.0);
            assertEquals(1.0, asset.get("MaxScale").getAsDouble(), 0.0);
        }

        @Test
        void pointsAtGeometryThatExistsInAnAllowedDirectory() throws IOException {
            String model = readJson(ASSET_PATH).get("Model").getAsString();

            assertTrue(model.endsWith(".blockymodel"), "MODEL_CHARACTER requires the blockymodel extension");
            assertTrue(
                    ALLOWED_COMMON_ROOTS.stream().anyMatch(model::startsWith),
                    model + " must sit under one of " + ALLOWED_COMMON_ROOTS);
            assertTrue(existsInPack(model), "Common/" + model + " must be packaged alongside the asset");
        }

        @Test
        void pointsAtATextureThatExistsInAnAllowedDirectory() throws IOException {
            String texture = readJson(ASSET_PATH).get("Texture").getAsString();

            assertTrue(texture.endsWith(".png"), "TEXTURE_CHARACTER requires the png extension");
            assertTrue(
                    ALLOWED_COMMON_ROOTS.stream().anyMatch(texture::startsWith),
                    texture + " must sit under one of " + ALLOWED_COMMON_ROOTS);
            assertTrue(existsInPack(texture), "Common/" + texture + " must be packaged alongside the asset");
        }
    }

    @Nested
    class TheGeometry {

        @Nonnull
        private JsonObject readModel() throws IOException {
            return readJson("/Common/" + readJson(ASSET_PATH).get("Model").getAsString());
        }

        @Test
        void parsesAsTheNodeTreeTheServersBoundsParserExpects() throws IOException {
            // BlockyModelBoundsParser reads root.nodes, then node.shape.settings.size
            // (`asset/type/model/BlockyModelBoundsParser.java:56,84,106`). A shape it cannot walk
            // yields a zero bounding box rather than an error.
            JsonArray nodes = readModel().getAsJsonArray("nodes");
            assertNotNull(nodes, "a blockymodel must have a nodes array");
            assertTrue(nodes.size() > 0, "an empty node list renders nothing");
        }

        @Test
        void givesEveryBoxAThreeComponentSize() throws IOException {
            List<JsonObject> boxes = collectBoxShapes(readModel().getAsJsonArray("nodes"));

            assertTrue(boxes.size() >= 9, "expected a body, four arms and four rotors; found " + boxes.size());
            for (JsonObject shape : boxes) {
                JsonObject size = shape.getAsJsonObject("settings").getAsJsonObject("size");
                for (String axis : List.of("x", "y", "z")) {
                    assertTrue(size.has(axis), "box size." + axis + " is required for a box shape");
                    assertTrue(size.get(axis).getAsDouble() > 0.0, "a zero-sized box renders nothing");
                }
            }
        }

        @Test
        void isRoughlyCentredOnTheOriginBecauseThatIsWhatTheIntegratorRotatesAbout() throws IOException {
            JsonObject hitBox = readJson(ASSET_PATH).getAsJsonObject("HitBox");
            double minX = hitBox.getAsJsonObject("Min").get("X").getAsDouble();
            double maxX = hitBox.getAsJsonObject("Max").get("X").getAsDouble();
            double minZ = hitBox.getAsJsonObject("Min").get("Z").getAsDouble();
            double maxZ = hitBox.getAsJsonObject("Max").get("Z").getAsDouble();

            // A quad's transform origin is its centre of mass. An off-centre hitbox would make the
            // drone appear to orbit a point beside itself as it rotates.
            assertEquals(0.0, minX + maxX, 1.0e-9, "hitbox must be symmetric in X");
            assertEquals(0.0, minZ + maxZ, 1.0e-9, "hitbox must be symmetric in Z");
        }

        private static List<JsonObject> collectBoxShapes(@Nonnull JsonArray nodes) {
            List<JsonObject> boxes = new java.util.ArrayList<>();
            for (JsonElement element : nodes) {
                JsonObject node = element.getAsJsonObject();
                JsonObject shape = node.getAsJsonObject("shape");
                if (shape != null && shape.has("type") && "box".equals(shape.get("type").getAsString())) {
                    boxes.add(shape);
                }
                JsonArray children = node.getAsJsonArray("children");
                if (children != null) {
                    boxes.addAll(collectBoxShapes(children));
                }
            }
            return boxes;
        }
    }
}
