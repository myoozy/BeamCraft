package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JBeamPartMergerTest {
    @Test
    void mergesAllNamedConfigurationFieldsWithoutAFieldWhitelist() {
        JsonObject base = object("""
                {
                  "slotType":"main",
                  "information":{"name":"root"},
                  "turbocharger":{
                    "inertia":0.2,
                    "wastegateStart":7,
                    "bovEnabled":true,
                    "pressurePSI":[[0,0],[100000,20]],
                    "futureField":{"source":"base"}
                  }
                }
                """);
        JsonObject ecu = object("""
                {
                  "slotType":"ecu",
                  "information":{"name":"ecu"},
                  "turbocharger":{
                    "$*inertia":2,
                    "wastegateStart":[19],
                    "bovEnabled":false,
                    "pressurePSI":[[0,-1],[120000,25]],
                    "futureField":{"source":"ecu"}
                  }
                }
                """);

        JsonObject merged = JBeamPartMerger.mergeParts(List.of(base, ecu));
        JsonObject turbo = merged.getAsJsonObject("turbocharger");
        assertEquals(0.4, turbo.get("inertia").getAsDouble(), 1e-9);
        assertEquals(19.0, turbo.getAsJsonArray("wastegateStart").get(0).getAsDouble(), 1e-9);
        assertFalse(turbo.get("bovEnabled").getAsBoolean());
        assertEquals(120000, turbo.getAsJsonArray("pressurePSI").get(1).getAsJsonArray().get(0).getAsInt());
        assertEquals("ecu", turbo.getAsJsonObject("futureField").get("source").getAsString());
        assertEquals("main", merged.get("slotType").getAsString());
        assertFalse(merged.has("information"));
    }

    @Test
    void appendsTableRowsWithoutDuplicatingHeaders() {
        JsonObject root = object("""
                {"powertrain":[["type","name","inputName","inputIndex"],["engine","mainEngine","dummy",0]]}
                """);
        JsonObject child = object("""
                {"powertrain":[["type","name","inputName","inputIndex"],["clutch","clutch","mainEngine",1]]}
                """);

        JsonArray powertrain = JBeamPartMerger.mergeParts(List.of(root, child)).getAsJsonArray("powertrain");
        assertEquals(3, powertrain.size());
        assertEquals("engine", powertrain.get(1).getAsJsonArray().get(0).getAsString());
        assertEquals("clutch", powertrain.get(2).getAsJsonArray().get(0).getAsString());
        assertTrue(powertrain.get(0).isJsonArray());
    }

    private static JsonObject object(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }
}
