package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** Regression tests for the official BeamNG NORMAL/BOUNDED beam property defaults. */
class JBeamBeamDefaultsTest {

    private static final float EPS = 1.0e-3f;

    @Test
    void propertyFreeNormalBeamGetsOfficialDefaults() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        JBeamParser.parseBeams(rows("""
                [
                  ["id1:", "id2:"],
                  ["a", "b"]
                ]
                """), vehicle, partEntry());

        BeamContainer beams = vehicle.normalBeams;
        assertEquals(1, beams.count);
        assertEquals(4_300_000.0f, beams.spring[0], EPS);
        assertEquals(580.0f, beams.damp[0], EPS);
        assertEquals(220_000.0f, beams.deform[0], EPS);
        assertEquals(Float.MAX_VALUE, beams.strength[0], EPS, "beamStrength is unbounded");
        assertEquals(Float.MAX_VALUE, beams.deformLimitStress[0], EPS,
                "deformLimitStress is unbounded");
        assertEquals(Float.MAX_VALUE, beams.maxDeform[0], EPS,
                "unbounded strength and hardening must not cap deformation");
        assertEquals(1.0f, beams.restLength[0], EPS, "beamPrecompression defaults to 1");
        assertEquals(0.0f, beams.precompTimeTotal[0], EPS,
                "beamPrecompressionTime defaults to 0");
    }

    @Test
    void propertyFreeBoundedBeamGetsOfficialDefaultsAndDampingFallbacks() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        JBeamParser.parseBeams(rows("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED"},
                  ["a", "b"]
                ]
                """), vehicle, partEntry());

        BoundedBeamContainer beams = vehicle.boundedBeams;
        assertEquals(1, beams.count);
        assertEquals(4_300_000.0f, beams.spring[0], EPS);
        assertEquals(580.0f, beams.damp[0], EPS);
        assertEquals(220_000.0f, beams.deform[0], EPS);
        assertEquals(Float.MAX_VALUE, beams.strength[0], EPS);
        assertEquals(1.0f, beams.shortBound[0], EPS);
        assertEquals(1.0f, beams.longBound[0], EPS);
        assertEquals(1.0f, beams.limitSpring[0], EPS, "beamLimitSpring defaults to 1");
        assertEquals(1.0f, beams.limitDamp[0], EPS, "beamLimitDamp defaults to 1");
        assertEquals(1.0f, beams.boundZone[0], EPS, "boundZone defaults to 1 meter");
        assertEquals(1.0f, beams.limitDampRebound[0], EPS,
                "beamLimitDampRebound falls back to beamLimitDamp");
        assertEquals(Float.MAX_VALUE, beams.dampVelocitySplit[0], EPS,
                "beamDampVelocitySplit is unbounded by default");
        assertEquals(580.0f, beams.dampFast[0], EPS, "beamDampFast falls back to beamDamp");
        assertEquals(580.0f, beams.dampRebound[0], EPS, "beamDampRebound falls back to beamDamp");
        assertEquals(580.0f, beams.dampReboundFast[0], EPS,
                "beamDampReboundFast falls back to beamDampRebound");
    }

    @Test
    void scopedSpringAndDampDoNotAliasTheLimitDefaults() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        JBeamParser.parseBeams(rows("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED", "beamSpring": 123456.0, "beamDamp": 42.0},
                  ["a", "b"]
                ]
                """), vehicle, partEntry());

        BoundedBeamContainer beams = vehicle.boundedBeams;
        assertEquals(123456.0f, beams.spring[0], EPS);
        assertEquals(42.0f, beams.damp[0], EPS);
        assertEquals(1.0f, beams.limitSpring[0], EPS,
                "an unspecified limit spring must not follow the scoped beamSpring");
        assertEquals(1.0f, beams.limitDamp[0], EPS,
                "an unspecified limit damp must not follow the scoped beamDamp");
        assertEquals(42.0f, beams.dampFast[0], EPS);
        assertEquals(42.0f, beams.dampRebound[0], EPS);
        assertEquals(42.0f, beams.dampReboundFast[0], EPS);
        assertEquals(Float.MAX_VALUE, beams.dampVelocitySplit[0], EPS);
    }

    @Test
    void explicitBoundedValuesWinOverDefaults() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        JBeamParser.parseBeams(rows("""
                [
                  ["id1:", "id2:", "beamType:"],
                  ["a", "b", {
                    "beamType": "|BOUNDED",
                    "beamSpring": 5000.0, "beamDamp": 50.0,
                    "beamDeform": 3000.0, "beamStrength": 4000.0,
                    "beamPrecompression": 1.05,
                    "beamShortBound": 0.6, "beamLongBound": 1.4, "boundZone": 0.25,
                    "beamLimitSpring": 7000.0, "beamLimitDamp": 80.0,
                    "beamLimitDampRebound": 85.0,
                    "beamDampVelocitySplit": 12.0, "beamDampFast": 90.0,
                    "beamDampRebound": 100.0, "beamDampReboundFast": 110.0
                  }]
                ]
                """), vehicle, partEntry());

        BoundedBeamContainer beams = vehicle.boundedBeams;
        assertEquals(1, beams.count);
        assertEquals(5000.0f, beams.spring[0], EPS);
        assertEquals(50.0f, beams.damp[0], EPS);
        assertEquals(3000.0f, beams.deform[0], EPS);
        assertEquals(4000.0f, beams.strength[0], EPS);
        assertEquals(1.05f, beams.targetRestLength[0], EPS);
        assertEquals(0.6f, beams.shortBound[0], EPS);
        assertEquals(1.4f, beams.longBound[0], EPS);
        assertEquals(0.25f, beams.boundZone[0], EPS);
        assertEquals(7000.0f, beams.limitSpring[0], EPS);
        assertEquals(80.0f, beams.limitDamp[0], EPS);
        assertEquals(85.0f, beams.limitDampRebound[0], EPS);
        assertEquals(12.0f, beams.dampVelocitySplit[0], EPS);
        assertEquals(90.0f, beams.dampFast[0], EPS);
        assertEquals(100.0f, beams.dampRebound[0], EPS);
        assertEquals(110.0f, beams.dampReboundFast[0], EPS);
    }

    @Test
    void emptyScopedValuesRestoreSchemaDefaults() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        JBeamParser.parseBeams(rows("""
                [
                  ["id1:", "id2:"],
                  {"beamType":"|BOUNDED", "beamSpring":1234, "beamDamp":45,
                   "beamDeform":67, "beamStrength":89,
                   "beamLimitSpring":321, "beamLimitDamp":54, "boundZone":0.5,
                   "beamLimitDampRebound":66,
                   "beamDampVelocitySplit":2, "beamDampFast":3,
                   "beamDampRebound":4, "beamDampReboundFast":5},
                  ["a", "b"],
                  {"beamSpring":"", "beamDamp":"", "beamDeform":"", "beamStrength":"",
                   "beamLimitSpring":"", "beamLimitDamp":"", "boundZone":"",
                   "beamLimitDampRebound":"",
                   "beamDampVelocitySplit":"", "beamDampFast":"",
                   "beamDampRebound":"", "beamDampReboundFast":""},
                  ["a", "b"]
                ]
                """), vehicle, partEntry());

        BoundedBeamContainer beams = vehicle.boundedBeams;
        assertEquals(2, beams.count);
        assertEquals(DEFAULT_NORMAL_SPRING, beams.spring[1], EPS);
        assertEquals(DEFAULT_NORMAL_DAMP, beams.damp[1], EPS);
        assertEquals(220_000.0f, beams.deform[1], EPS);
        assertEquals(Float.MAX_VALUE, beams.strength[1], EPS);
        assertEquals(1.0f, beams.limitSpring[1], EPS);
        assertEquals(1.0f, beams.limitDamp[1], EPS);
        assertEquals(1.0f, beams.boundZone[1], EPS);
        assertEquals(1.0f, beams.limitDampRebound[1], EPS,
                "an empty beamLimitDampRebound restores the beamLimitDamp fallback");
        assertEquals(Float.MAX_VALUE, beams.dampVelocitySplit[1], EPS);
        assertEquals(DEFAULT_NORMAL_DAMP, beams.dampFast[1], EPS);
        assertEquals(DEFAULT_NORMAL_DAMP, beams.dampRebound[1], EPS);
        assertEquals(DEFAULT_NORMAL_DAMP, beams.dampReboundFast[1], EPS);
    }

    @Test
    void emptyBeamTypeRestoresNormalType() {
        SoftBodyVehicle vehicle = vehicleWithTwoNodes();
        JBeamParser.parseBeams(rows("""
                [
                  ["id1:", "id2:"],
                  {"beamType":"|BOUNDED"},
                  ["a", "b"],
                  {"beamType":""},
                  ["a", "b"]
                ]
                """), vehicle, partEntry());

        assertEquals(1, vehicle.boundedBeams.count);
        assertEquals(1, vehicle.normalBeams.count);
    }

    private static final float DEFAULT_NORMAL_SPRING = 4_300_000.0f;
    private static final float DEFAULT_NORMAL_DAMP = 580.0f;

    private static JsonArray rows(String json) {
        return JsonParser.parseString(json).getAsJsonArray();
    }

    private static SoftBodyVehicle vehicleWithTwoNodes() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0.0f, 0.0f));
        vehicle.addNode(node("b", 1.0f, 0.0f));
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x, float y) {
        return new PhysicsSpecs.NodeSpec(name, x, y, 0.0f,
                1.0f, 1.0f, 1.0f, 0, false, false, List.of());
    }

    private static JBeamAssembler.PartEntry partEntry() {
        return new JBeamAssembler.PartEntry(
                new JsonObject(), 1, "test", new JBeamAssembler.TransformContext(), Map.of());
    }
}
