package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

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

    private static JsonObject part(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }
}
