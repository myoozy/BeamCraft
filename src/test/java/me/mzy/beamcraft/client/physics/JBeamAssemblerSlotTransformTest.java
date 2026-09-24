package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JBeamAssemblerSlotTransformTest {

    @Test
    void acceptsEmptyStringNodeOffsetSentinel() {
        JsonObject root = part("""
                {
                  "slots2": [
                    ["type", "default"],
                    ["engine", "engine_part", {"nodeOffset": ""}]
                  ]
                }
                """);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "root",
                Map.of(),
                Map.of("root", root, "engine_part", new JsonObject()),
                new SoftBodyVehicle(null)));
    }

    @Test
    void resolvesExportedVehicleRootByMainSlotType() {
        JsonObject exportedRoot = part("""
                {
                  "slotType": "main",
                  "nodes": [
                    ["id", "posX", "posY", "posZ"],
                    ["automation_node", 1, 2, 3]
                  ]
                }
                """);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "3yc_q",
                Map.of(),
                Map.of("Camso_3yc_q_core", exportedRoot),
                vehicle));
        assertEquals(1, vehicle.nodes.count);
        assertTrue(vehicle.nodes.nameToIndex.containsKey("automation_node"));
    }

    @Test
    void missingRootPartFailsInsteadOfProducingEmptyVehicle() {
        assertFalse(new JBeamAssembler().assembleVehicle(
                "missing",
                Map.of(),
                Map.of("unrelated", new JsonObject()),
                new SoftBodyVehicle(null)));
    }

    @Test
    void installedEtk800TtSportDctConfigurationAssembles() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(
                corpusRoot,
                "etk800",
                "846x_ttsport_plus_DCT.pc",
                registry,
                config);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "etk800", config, registry, new SoftBodyVehicle(null)));
    }

    @Test
    void installedAutomationExportConfigurationAssembles() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        String mods = System.getenv("BEAMCRAFT_AUTOMATION_MODS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        Assumptions.assumeTrue(mods != null && !mods.isBlank());
        File corpusRoot = new File(corpus);
        File modsRoot = new File(mods);
        Assumptions.assumeTrue(corpusRoot.isDirectory());
        Assumptions.assumeTrue(modsRoot.isDirectory());

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        assertTrue(JBeamLoader.loadVehicle(
                List.of(corpusRoot, modsRoot),
                "3yc_q",
                "564a4.pc",
                registry,
                config));
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "3yc_q", config, registry, vehicle));
        assertTrue(vehicle.nodes.count > 0);
    }

    private static JsonObject part(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }
}
