package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.material.RelaxedJson;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end coverage of the assembled-part lifecycle: JBeam source text ->
 * beam {@code name} preservation -> controller registration after the named
 * beams exist -> runtime mode commands.
 *
 * <p>The part is a trimmed copy of the stock ETK800
 * {@code etk800_shock_R_wide_adaptive} declaration.
 */
class AdaptiveDamperAssemblyTest {

    private static final float TOL = 1.0e-2f;
    private static final String CONTROLLER = "adaptiveRearDamper";

    private static final String ADAPTIVE_SHOCK_PART = """
            {
              "nodes": [
                ["id", "posX", "posY", "posZ"],
                ["rh1r", 0.0, 0.0, 0.0],
                ["r1rr", 0.0, 0.0, -0.4],
                ["rh1l", 1.0, 0.0, 0.0],
                ["r1ll", 1.0, 0.0, -0.4]
              ],
              "beams": [
                ["id1:", "id2:"],
                {"beamPrecompression":1, "beamType":"|BOUNDED", "beamLongBound":1, "beamShortBound":1},
                {"beamDeform":15000,"beamStrength":150000},
                {"beamLimitSpring":0,"beamLimitDamp":0},
                {"beamSpring":0,"beamDamp":6000},
                ["rh1r", "r1rr", {"name":"shock_RR", "beamDampFast":1500,"beamDampRebound":8500,
                                  "beamDampReboundFast":2833,"beamDampVelocitySplit":0.25,"dampCutoffHz":500}],
                ["rh1l", "r1ll", {"name":"shock_RL", "beamDampFast":1500,"beamDampRebound":8500,
                                  "beamDampReboundFast":2833,"beamDampVelocitySplit":0.25,"dampCutoffHz":500}]
              ],
              "controller": [
                ["fileName"],
                ["drivingDynamics/actuators/adaptiveDampers" {"name":"adaptiveRearDamper", "dampBeamNames":["shock_RR", "shock_RL"]}]
              ],
              "adaptiveRearDamper": {
                "modes": [
                  ["name",    "beamDampCoef", "beamDampFastCoef", "beamDampReboundCoef", "beamDampReboundFastCoef","beamDampVelocitySplitCoef"],
                  ["soft",    0.7,            1,                  0.65,                  1,                        0.7],
                  ["regular", 1,              1,                  1,                     1,                        1],
                  ["hard",    1.5,            1,                  1.4,                   1,                        1.2]
                ]
              }
            }
            """;

    private static SoftBodyVehicle assembleAdaptiveShockPart() {
        // The real pipeline cleans relaxed JBeam syntax (the controller row omits a
        // comma before its inline options object) before the assembler sees it.
        JsonObject root = RelaxedJson.parse(ADAPTIVE_SHOCK_PART);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle(
                "root", Map.of(), Map.of("root", root), vehicle));
        return vehicle;
    }

    @Test
    void controllerIsRegisteredOnlyOnceTheNamedBeamsExist() {
        SoftBodyVehicle vehicle = assembleAdaptiveShockPart();

        assertEquals(2, vehicle.boundedBeams.count);
        assertArrayEquals(new int[]{0}, vehicle.boundedBeams.indicesForName("shock_RR"));
        assertArrayEquals(new int[]{1}, vehicle.boundedBeams.indicesForName("shock_RL"));

        assertTrue(vehicle.adaptiveDampers.hasController(CONTROLLER));
        assertEquals(2, vehicle.adaptiveDampers.controller(CONTROLLER).beamIndices().length);
    }

    @Test
    void theAuthoredDampingSurvivesParsingAndAssembly() {
        SoftBeamView beams = new SoftBeamView(assembleAdaptiveShockPart());
        assertEquals(6000.0f, beams.damp(0), TOL);
        assertEquals(1500.0f, beams.dampFast(0), TOL);
        assertEquals(8500.0f, beams.dampRebound(0), TOL);
        assertEquals(2833.0f, beams.dampReboundFast(0), TOL);
        assertEquals(0.25f, beams.dampVelocitySplit(0), TOL);
        assertEquals(6000.0f, beams.authoredDamp(0), TOL);
        assertEquals(8500.0f, beams.authoredDampRebound(0), TOL);
    }

    @Test
    void regularIsTheDefaultAndModeCommandsReachTheBeams() {
        SoftBodyVehicle vehicle = assembleAdaptiveShockPart();
        assertEquals("regular", vehicle.adaptiveDampers.currentMode(CONTROLLER));

        assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard"));
        SoftBeamView beams = new SoftBeamView(vehicle);
        assertEquals(9000.0f, beams.damp(0), TOL);
        assertEquals(11900.0f, beams.dampRebound(0), TOL);
        assertEquals(0.3f, beams.dampVelocitySplit(0), TOL);

        assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "soft"));
        assertEquals(4200.0f, beams.damp(0), TOL);
        assertEquals(5525.0f, beams.dampRebound(0), TOL);
        assertEquals(0.175f, beams.dampVelocitySplit(0), TOL);
    }

    @Test
    void queuedCommandsAreAppliedByThePhysicsSubstep() {
        SoftBodyVehicle vehicle = assembleAdaptiveShockPart();
        assertTrue(vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "soft"));

        vehicle.solveInternalForces(1.0f / PhysicsWorld.invPhysicsDT, 1.0f);

        assertEquals("soft", vehicle.adaptiveDampers.currentMode(CONTROLLER));
        assertEquals(4200.0f, new SoftBeamView(vehicle).damp(0), TOL);
    }

    @Test
    void unknownModesAreRejectedAndResetKeepsTheSelection() {
        SoftBodyVehicle vehicle = assembleAdaptiveShockPart();
        assertFalse(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "missing"));
        assertFalse(vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "missing"));

        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        float hard = new SoftBeamView(vehicle).damp(0);
        vehicle.reset();
        assertEquals("hard", vehicle.adaptiveDampers.currentMode(CONTROLLER));
        assertEquals(hard, new SoftBeamView(vehicle).damp(0), TOL);
    }

    @Test
    void aPartWithoutControllersAssemblesWithNoActuators() {
        JsonObject plain = JsonParser.parseString("""
                {
                  "nodes": [["id","posX","posY","posZ"], ["a",0,0,0], ["b",1,0,0]],
                  "beams": [["id1:","id2:"], ["a","b"]]
                }
                """).getAsJsonObject();
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("root", Map.of(), Map.of("root", plain), vehicle));

        assertTrue(vehicle.adaptiveDampers.controllerNames().isEmpty());
        assertFalse(vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "hard"));
        assertArrayEquals(new int[0], vehicle.boundedBeams.indicesForName("shock_RR"));
    }

    /** Small read-only view so assertions read like BeamNG property names. */
    private record SoftBeamView(BoundedBeamContainer beams) {
        SoftBeamView(SoftBodyVehicle vehicle) {
            this(vehicle.boundedBeams);
        }

        float damp(int i) { return beams.damp[i]; }
        float dampFast(int i) { return beams.dampFast[i]; }
        float dampRebound(int i) { return beams.dampRebound[i]; }
        float dampReboundFast(int i) { return beams.dampReboundFast[i]; }
        float dampVelocitySplit(int i) { return beams.dampVelocitySplit[i]; }
        float authoredDamp(int i) { return beams.authoredDamp[i]; }
        float authoredDampRebound(int i) { return beams.authoredDampRebound[i]; }
    }
}
