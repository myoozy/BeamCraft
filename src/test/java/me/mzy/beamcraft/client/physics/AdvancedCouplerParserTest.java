package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AdvancedCouplerParserTest {

    @Test
    void parsesModernControllerPairByHeaderName() {
        JsonObject part = part("""
                {
                  "controller": [
                    ["fileName"],
                    ["advancedCouplerControl", {"name":"door_FR_coupler"}]
                  ],
                  "door_FR_coupler": {
                    "couplerNodes": [
                      ["cid1", "cid2", "autoCouplingStrength", "autoCouplingRadius",
                       "autoCouplingLockRadius", "autoCouplingSpeed", "couplingStartRadius", "breakGroup"],
                      ["bodyLatch", "doorLatch", 55000, 0.01, 0.005, 0.2, 0.2, "doorFR_latch"]
                    ]
                  }
                }
                """);

        CouplerRegistry registry = new CouplerRegistry();
        JBeamParser.parseAdvancedCouplers(part, Map.of(), registry);

        assertEquals(1, registry.directDefinitions.size());
        CouplerRegistry.DirectCouplerDef pair = registry.directDefinitions.getFirst();
        assertEquals("door_FR_coupler", pair.controllerName);
        assertEquals("bodyLatch", pair.node1);
        assertEquals("doorLatch", pair.node2);
        assertEquals(55000.0, pair.strength, 1e-6);
        assertEquals(0.005, pair.lockRadius, 1e-6);
        assertEquals(0.2, pair.latchSpeed, 1e-6);
        assertEquals(0.2, pair.startRadius, 1e-6);
        assertEquals("doorFR_latch", pair.breakGroup);
    }

    @Test
    void assemblerCreatesBreakGroupAwareSpawnConstraint() {
        JsonObject root = part("""
                {
                  "nodes": [
                    ["id", "posX", "posY", "posZ"],
                    ["bodyLatch", 0, 0, 0],
                    ["doorLatch", 0.1, 0, 0]
                  ],
                  "controller": [
                    ["fileName"],
                    ["advancedCouplerControl", {"name":"door_FR_coupler"}]
                  ],
                  "door_FR_coupler": {
                    "couplerNodes": [
                      ["cid1", "cid2", "autoCouplingStrength", "autoCouplingRadius",
                       "autoCouplingLockRadius", "autoCouplingSpeed", "couplingStartRadius", "breakGroup"],
                      ["bodyLatch", "doorLatch", 55000, 0.01, 0.005, 0.2, 0.2, "doorFR_latch"]
                    ]
                  }
                }
                """);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);

        assertTrue(new JBeamAssembler().assembleVehicle(
                "root", Map.of(), Map.of("root", root), vehicle));

        assertEquals(0, vehicle.normalBeams.count);
        assertEquals(1, vehicle.couplers.count);
        assertEquals(55000.0f, vehicle.couplers.strength[0], 1e-3f);
        assertEquals(0.005f, vehicle.couplers.lockRadius[0], 1e-6f);
        assertEquals(CouplerContainer.LATCHING, vehicle.couplers.state[0]);

        vehicle.triggerBreakGroup("doorFR_latch");
        assertEquals(CouplerContainer.BROKEN, vehicle.couplers.state[0]);
    }

    @Test
    void legacyNodeCouplerPropertiesInheritFromModifierRows() {
        JsonObject part = part("""
                {
                  "nodes": [
                    ["id", "posX", "posY", "posZ"],
                    {"couplerTag":"door_latch", "couplerStartRadius":0.1, "couplerStrength":1234},
                    ["coupler", 0, 0, 0],
                    {"couplerTag":"", "tag":"door_latch"},
                    ["tag", 0, 0, 0]
                  ]
                }
                """);
        CouplerRegistry registry = new CouplerRegistry();
        JBeamAssembler.PartEntry entry = new JBeamAssembler.PartEntry(
                part, 1, "root", new JBeamAssembler.TransformContext(), new HashMap<>());

        JBeamParser.parseNodes(part.getAsJsonArray("nodes"), new SoftBodyVehicle(null), entry, registry);

        assertEquals(2, registry.definitions.size());
        assertEquals("door_latch", registry.definitions.get(0).couplerTag);
        assertEquals(1234.0, registry.definitions.get(0).strength, 1e-6);
        assertEquals("door_latch", registry.definitions.get(1).tag);
        assertEquals("", registry.definitions.get(1).couplerTag);
    }

    private static JsonObject part(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }
}
