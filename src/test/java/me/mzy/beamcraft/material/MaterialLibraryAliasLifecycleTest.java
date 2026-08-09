package me.mzy.beamcraft.client.material;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end lifecycle tests for the static glowMap alias index inside
 * {@link MaterialLibrary}: folder-based and ZIP-based vehicles are scanned for
 * {@code *.jbeam} glowMap aliases exactly like materials, lookups resolve
 * through the alias, aliases are scoped per namespace, collision resolution is
 * deterministic, a malformed JBeam file degrades safely, and
 * {@link MaterialLibrary#releaseMaterials} clears the namespace's alias index.
 *
 * <p>Each test builds its own vehicle tree in a {@link TempDir} and releases it
 * in teardown, so the in-process static library state does not leak between
 * tests.
 */
class MaterialLibraryAliasLifecycleTest {

    private static final String[] TEST_NAMESPACES = {"pickup", "sunburst2", "zipcar", "collide"};

    @TempDir
    Path root;

    @AfterEach
    void releaseRemainingNamespaces() {
        for (String ns : TEST_NAMESPACES) {
            while (true) {
                // Ref-counted: release until the namespace is actually removed.
                boolean before = MaterialLibrary.getAliasNamespaces().contains(ns)
                        || MaterialLibrary.getLoadedNamespaces().contains(ns);
                if (!before) {
                    break;
                }
                MaterialLibrary.releaseMaterials(ns);
            }
        }
    }

    private static final String LIGHTGLASS_MATERIALS = """
            {
              "lightglass": {
                "name": "pickup_lightglass",
                "mapTo": "pickup_lightglass",
                "class": "Material",
                "translucent": true,
                "translucentBlendOp": "None",
                "Stages": [
                  {
                    "baseColorMap": "/vehicles/common/pickup/pickup_lightglass_b.color.png",
                    "opacityMap": "/vehicles/common/pickup/pickup_lightglass_o.data.png"
                  }
                ]
              }
            }
            """;

    /** Pickup-style relaxed JBeam with comments/trailing commas and a glowMap. */
    private static final String PICKUP_JBEAM = """
            {
            "pickup_body":{
                "information":{"authors":"BeamNG","name":"Body","value":100,},
                // lamp covers, relaxed JSON dialect
                "glowMap":{
                    "pickup_lowbeamglass":{"simpleFunction":"lowbeam_filament", "off":"pickup_lightglass", "on":"pickup_lightglass_on", "materialEmissiveScaling":{"on_max":1.00}},
                    "pickup_taillightglass_R":{"simpleFunction":"lowhighBrakeSignal_R_filament", "off":"pickup_lightglass", "on":"pickup_lightglass_on",},
                },
            },
            }
            """;

    private static final String SUNBURST_JBEAM = """
            {
            "sunburst2_headlight":{
                "glowMap":{
                    "sunburst2_headlightglass":{"simpleFunction":{"lowbeam_filament":0.49,"highbeam_filament":1}, "off":"sunburst2_glass", "on":"sunburst2_glass_on", "on_intense":"sunburst2_glass_on_intense", "materialEmissiveScaling":{"on_max":1.00}},
                },
            },
            }
            """;

    private static void write(Path file, String content) throws IOException {
        Files.createDirectories(file.getParent());
        Files.writeString(file, content, StandardCharsets.UTF_8);
    }

    private static void writeZip(Path zip, java.util.Map<String, String> entries) throws IOException {
        try (ZipOutputStream out = new ZipOutputStream(Files.newOutputStream(zip))) {
            for (java.util.Map.Entry<String, String> entry : entries.entrySet()) {
                out.putNextEntry(new ZipEntry(entry.getKey()));
                out.write(entry.getValue().getBytes(StandardCharsets.UTF_8));
                out.closeEntry();
            }
        }
    }

    // ------------------------------------------------------------------
    // Folder-based vehicles
    // ------------------------------------------------------------------

    @Test
    void folderVehicleResolvesThroughGlowMapAlias() throws IOException {
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), LIGHTGLASS_MATERIALS);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"), PICKUP_JBEAM);

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");

        assertTrue(MaterialLibrary.getAliasNamespaces().contains("pickup"));
        MaterialDefinition def = MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass");
        assertNotNull(def, "the DAE material must resolve through the glowMap alias");
        assertEquals("pickup_lightglass", def.mapTo);
        assertTrue(def.translucent, "the lights-off glass material must keep its translucent flag");

        MaterialDefinition tail = MaterialLibrary.getMaterial("pickup", "pickup_taillightglass_r");
        assertNotNull(tail);
        assertEquals("pickup_lightglass", tail.mapTo);
    }

    @Test
    void folderVehicleAliasesSurviveDirectMaterialLookupFallback() throws IOException {
        // A DAE key that exists directly in the vehicle index must NOT be
        // redirected by an alias (direct mapTo wins).
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), """
                {
                  "body": { "name": "pickup_body", "mapTo": "pickup_body",
                            "Stages": [ { "baseColorMap": "/vehicles/pickup/body_d.png" } ] },
                  "pickup_lightglass": { "name": "pickup_lightglass", "mapTo": "pickup_lightglass",
                                         "translucent": true,
                                         "Stages": [ { "baseColorMap": "/vehicles/pickup/lightglass_d.png" } ] }
                }
                """);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"), PICKUP_JBEAM);

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");

        // The direct key exists -> resolves to pickup_body, not the alias target.
        assertEquals("pickup_body", MaterialLibrary.getMaterial("pickup", "pickup_body").mapTo);
        assertEquals("pickup_lightglass", MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass").mapTo);
    }

    // ------------------------------------------------------------------
    // ZIP-based vehicles
    // ------------------------------------------------------------------

    @Test
    void zipVehicleResolvesThroughGlowMapAlias() throws IOException {
        writeZip(root.resolve("zipcar.zip"), java.util.Map.of(
                "vehicles/zipcar/main.materials.json", LIGHTGLASS_MATERIALS,
                "vehicles/zipcar/body.jbeam", PICKUP_JBEAM));

        MaterialLibrary.requireMaterials(root.toFile(), "zipcar");

        MaterialDefinition def = MaterialLibrary.getMaterial("zipcar", "pickup_lowbeamglass");
        assertNotNull(def, "a ZIP-based vehicle must index glowMap aliases too");
        assertEquals("pickup_lightglass", def.mapTo);
        assertTrue(MaterialLibrary.getAliasNamespaces().contains("zipcar"));
    }

    @Test
    void zipVehicleIgnoresMacOsxJunkEntries() throws IOException {
        writeZip(root.resolve("zipcar.zip"), java.util.Map.of(
                "vehicles/zipcar/main.materials.json", LIGHTGLASS_MATERIALS,
                "vehicles/zipcar/body.jbeam", PICKUP_JBEAM,
                "__MACOSX/vehicles/zipcar/._body.jbeam", PICKUP_JBEAM));

        MaterialLibrary.requireMaterials(root.toFile(), "zipcar");
        assertEquals("pickup_lightglass",
                MaterialLibrary.getMaterial("zipcar", "pickup_lowbeamglass").mapTo);
    }

    // ------------------------------------------------------------------
    // Scoping
    // ------------------------------------------------------------------

    @Test
    void aliasesDoNotLeakAcrossVehicleNamespaces() throws IOException {
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), LIGHTGLASS_MATERIALS);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"), PICKUP_JBEAM);
        write(root.resolve("sunburst2/vehicles/sunburst2/main.materials.json"), """
                {
                  "glass": { "name": "sunburst2_glass", "mapTo": "sunburst2_glass",
                             "translucent": true,
                             "Stages": [ { "baseColorMap": "/vehicles/sunburst2/glass_d.png" } ] }
                }
                """);
        write(root.resolve("sunburst2/vehicles/sunburst2/body.jbeam"), SUNBURST_JBEAM);

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");
        MaterialLibrary.requireMaterials(root.toFile(), "sunburst2");

        assertEquals("pickup_lightglass",
                MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass").mapTo);
        assertEquals("sunburst2_glass",
                MaterialLibrary.getMaterial("sunburst2", "sunburst2_headlightglass").mapTo);

        // No leak: each namespace only sees its own aliases.
        assertNull(MaterialLibrary.getMaterial("pickup", "sunburst2_headlightglass"));
        assertNull(MaterialLibrary.getMaterial("sunburst2", "pickup_lowbeamglass"));
    }

    // ------------------------------------------------------------------
    // Collision determinism + malformed files
    // ------------------------------------------------------------------

    @Test
    void duplicateAliasAcrossFilesResolvesDeterministicallyToSortedLastFile() throws IOException {
        write(root.resolve("collide/vehicles/collide/main.materials.json"), """
                {
                  "mat_a": { "name": "mat_a", "mapTo": "mat_a", "Stages": [ { "baseColorMap": "/a.png" } ] },
                  "mat_b": { "name": "mat_b", "mapTo": "mat_b", "Stages": [ { "baseColorMap": "/b.png" } ] }
                }
                """);
        // a.jbeam sorts before b.jbeam, so b wins deterministically.
        write(root.resolve("collide/vehicles/collide/a.jbeam"),
                "{\"part\":{\"glowMap\":{\"shared_light\":{\"off\":\"mat_a\"}}}}");
        write(root.resolve("collide/vehicles/collide/b.jbeam"),
                "{\"part\":{\"glowMap\":{\"shared_light\":{\"off\":\"mat_b\"}}}}");

        MaterialLibrary.requireMaterials(root.toFile(), "collide");

        assertEquals("mat_b", MaterialLibrary.getMaterial("collide", "shared_light").mapTo,
                "the alphabetically-last scanned file must win for a duplicate alias key");
    }

    @Test
    void malformedJBeamDoesNotBlockMaterialIndexing() throws IOException {
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), LIGHTGLASS_MATERIALS);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"), PICKUP_JBEAM);
        write(root.resolve("pickup/vehicles/pickup/broken.jbeam"), "this is { not json, [at all] ::: ");

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");

        // The good file's aliases still indexed; the malformed one contributed nothing.
        assertEquals("pickup_lightglass",
                MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass").mapTo);
        assertNotNull(MaterialLibrary.getMaterial("pickup", "pickup_taillightglass_r"));
    }

    @Test
    void jbeamWithoutGlowMapStillIndexesMaterials() throws IOException {
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), LIGHTGLASS_MATERIALS);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"),
                "{\"pickup_body\":{\"nodes\":[[\"id\",\"posX\"],[\"n0\",0,0,0]],\"beams\":[[\"id1:\",\"id2:\"]]}}");

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");
        assertNotNull(MaterialLibrary.getMaterial("pickup", "pickup_lightglass"),
                "a JBeam without a glowMap must not disturb material indexing");
        assertNull(MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass"));
    }

    // ------------------------------------------------------------------
    // Lifecycle / cleanup
    // ------------------------------------------------------------------

    @Test
    void releaseClearsTheNamespaceAliasIndex() throws IOException {
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), LIGHTGLASS_MATERIALS);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"), PICKUP_JBEAM);

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");
        assertTrue(MaterialLibrary.getAliasNamespaces().contains("pickup"));
        assertNotNull(MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass"));

        MaterialLibrary.releaseMaterials("pickup");

        assertFalse(MaterialLibrary.getAliasNamespaces().contains("pickup"));
        assertFalse(MaterialLibrary.getLoadedNamespaces().contains("pickup"));
        assertNull(MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass"),
                "after release the alias must no longer resolve");
    }

    @Test
    void referenceCountingKeepsAliasesUntilLastInstanceReleases() throws IOException {
        write(root.resolve("pickup/vehicles/pickup/main.materials.json"), LIGHTGLASS_MATERIALS);
        write(root.resolve("pickup/vehicles/pickup/body.jbeam"), PICKUP_JBEAM);

        MaterialLibrary.requireMaterials(root.toFile(), "pickup");
        MaterialLibrary.requireMaterials(root.toFile(), "pickup");
        MaterialLibrary.releaseMaterials("pickup");

        assertTrue(MaterialLibrary.getAliasNamespaces().contains("pickup"),
                "one release of two references must keep the index alive");
        assertNotNull(MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass"));

        MaterialLibrary.releaseMaterials("pickup");
        assertFalse(MaterialLibrary.getAliasNamespaces().contains("pickup"));
        assertNull(MaterialLibrary.getMaterial("pickup", "pickup_lowbeamglass"));
    }

    @Test
    void releaseIsIdempotentForUnknownNamespace() {
        MaterialLibrary.releaseMaterials("does_not_exist"); // must not throw
        assertFalse(MaterialLibrary.getAliasNamespaces().contains("does_not_exist"));
    }
}
