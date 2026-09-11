package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JBeamDeformGroupTest {

    @Test
    void parserPreservesScopedAndInlineBeamDeformProperties() {
        SoftBodyVehicle vehicle = vehicleWithThreeNodes();
        JsonArray beams = JsonParser.parseString("""
                [
                  ["id1:", "id2:"],
                  {"deformGroup":["glass", "lights"], "deformationTriggerRatio":0.05},
                  ["a", "b"],
                  ["a", "c", {"deformGroup":"inline", "deformationTriggerRatio":"$ratio"}],
                  {"deformGroup":""},
                  ["b", "c"]
                ]
                """).getAsJsonArray();

        JBeamParser.parseBeams(beams, vehicle, partEntry(Map.of("ratio", 0.125)));

        BeamContainer parsed = vehicle.normalBeams;
        assertEquals(3, parsed.count);
        assertEquals(List.of("glass", "lights"), parsed.assignedDeformGroups[0]);
        assertEquals(0.05f, parsed.deformationTriggerRatio[0], 1.0e-6f);
        assertEquals(List.of("inline"), parsed.assignedDeformGroups[1]);
        assertEquals(0.125f, parsed.deformationTriggerRatio[1], 1.0e-6f);
        assertNull(parsed.assignedDeformGroups[2]);
    }

    @Test
    void flexbodyParserRetainsDamageMetadataWithoutSwitchingMaterials() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        JsonArray flexbodies = JsonParser.parseString("""
                [
                  ["mesh", "[group]:", "nonFlexMaterials"],
                  {"deformGroup":"windshield", "deformMaterialBase":"glass", "deformMaterialDamaged":"glass_dmg"},
                  ["bx_windshield", ["windshield"]],
                  ["bx_door_glass", ["door"], [], {
                    "deformGroup":"sideglass",
                    "deformMaterialBase":"sideglass_base",
                    "deformMaterialDamaged":"sideglass_dmg"
                  }],
                  {"deformGroup":"", "deformMaterialBase":"", "deformMaterialDamaged":""},
                  ["bx_body", ["body"]]
                ]
                """).getAsJsonArray();

        JBeamParser.parseFlexbodies(flexbodies, vehicle, "bx", partEntry(Map.of()));

        FlexbodyContainer parsed = vehicle.flexbodies;
        assertEquals(3, parsed.meshCount);
        assertEquals("windshield", parsed.deformGroup[0]);
        assertEquals("glass", parsed.deformMaterialBase[0]);
        assertEquals("glass_dmg", parsed.deformMaterialDamaged[0]);
        assertEquals("sideglass", parsed.deformGroup[1]);
        assertEquals("sideglass_base", parsed.deformMaterialBase[1]);
        assertEquals("sideglass_dmg", parsed.deformMaterialDamaged[1]);
        assertEquals("", parsed.deformGroup[2]);
    }

    @Test
    void strainTriggersOnceAndResetClearsTheLatchedState() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        vehicle.addBeam(beam(List.of("windshield"), 0.10f, Float.MAX_VALUE));

        vehicle.nodes.posX[1] = 1.09f;
        vehicle.updateDeformGroupTriggers();
        assertFalse(vehicle.isDeformGroupTriggered("windshield"),
                "strain below the configured threshold must not trigger damage");

        vehicle.nodes.posX[1] = 1.11f;
        vehicle.updateDeformGroupTriggers();
        assertTrue(vehicle.isDeformGroupTriggered("windshield"));
        assertTrue(vehicle.normalBeams.deformGroupTriggered[0]);

        vehicle.nodes.posX[1] = 1.0f;
        vehicle.updateDeformGroupTriggers();
        assertTrue(vehicle.isDeformGroupTriggered("windshield"), "damage must remain latched");

        vehicle.reset();
        assertFalse(vehicle.isDeformGroupTriggered("windshield"));
        assertFalse(vehicle.normalBeams.deformGroupTriggered[0]);
    }

    @Test
    void breakingBeamTriggersDeformGroupWithoutFiniteRatio() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        vehicle.addBeam(beam(List.of("sideglass"), Float.POSITIVE_INFINITY, 1.0f));

        vehicle.breakBeamAt(vehicle.normalBeams, 0);

        assertTrue(vehicle.normalBeams.broken[0]);
        assertTrue(vehicle.isDeformGroupTriggered("sideglass"));
    }

    private static SoftBodyVehicle vehicleWithTwoNodes() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0.0f, 0.0f));
        vehicle.addNode(node("b", 1.0f, 0.0f));
        return vehicle;
    }

    private static SoftBodyVehicle vehicleWithThreeNodes() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        vehicle.addNode(node("c", 0.0f, 1.0f));
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x, float y) {
        return new PhysicsSpecs.NodeSpec(name, x, y, 0.0f,
                1.0f, 1.0f, 1.0f, 0, false, false, List.of());
    }

    private static PhysicsSpecs.BeamSpec beam(List<String> deformGroups,
                                               float triggerRatio, float strength) {
        return new PhysicsSpecs.BeamSpec(
                BeamContainer.BEAM_NORMAL, "a", "b", null,
                deformGroups, triggerRatio,
                List.of(), 0, false,
                100.0f, 0.0f, -1.0f, Float.MAX_VALUE, strength,
                1.0f, 0.0f, false, 0.0f,
                1.0f, 1.0f, -1.0f, -1.0f, 1.0f,
                100.0f, 0.0f, -1.0f, -1.0f, -1.0f, -1.0f,
                -1.0f, -1.0f, 100.0f, 0.0f, 0.0f,
                Float.MAX_VALUE);
    }

    private static JBeamAssembler.PartEntry partEntry(Map<String, Double> variables) {
        return new JBeamAssembler.PartEntry(
                new JsonObject(), 1, "test", new JBeamAssembler.TransformContext(), variables);
    }
}
