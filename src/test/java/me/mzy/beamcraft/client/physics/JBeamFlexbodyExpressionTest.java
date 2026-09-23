package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JBeamFlexbodyExpressionTest {

    @Test
    void resolvesVehicleWideComponentObjectForFlexbodyTransform() {
        JsonObject root = part("""
                {
                  "components": {
                    "bodyType": {"hatch": false},
                    "rearSuspensionOffset": {"x": 0, "y": 0.11, "z": 0}
                  },
                  "slots": [
                    ["type", "default"],
                    ["rear_suspension", "rally_rear"]
                  ]
                }
                """);
        JsonObject suspension = part("""
                {
                  "slotType": "rear_suspension",
                  "flexbodies": [
                    ["mesh", "[group]:", "nonFlexMaterials"],
                    ["$=$components.bodyType.hatch == true and 'hatch_subframe' or 'sedan_subframe'",
                     ["rear_subframe"], [], {"pos":"$=$components.rearSuspensionOffset"}]
                  ]
                }
                """);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "root",
                Map.of(),
                Map.of("root", root, "rally_rear", suspension),
                vehicle));
        assertEquals(1, vehicle.flexbodies.meshCount);
        assertEquals("sedan_subframe", vehicle.flexbodies.meshName[0]);
        assertEquals(0.11, vehicle.flexbodies.posY[0], 1e-9);
    }

    @Test
    void installedVivaceRallyAsphaltConfigurationAssembles() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        assertTrue(JBeamLoader.loadVehicle(
                corpusRoot,
                "vivace",
                "ardente_rally_asphalt.pc",
                registry,
                config));
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "vivace", config, registry, vehicle));
        assertTrue(vehicle.nodes.count > 0);
        assertTrue(vehicle.flexbodies.meshCount > 0);
    }

    private static JsonObject part(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }
}
