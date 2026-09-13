package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Opt-in regression test against an installed BeamNG corpus.
 *
 * <p>Uses the stock ETK800 ttSport+ configuration
 * ({@code 846x_ttsport_plus_DCT.pc}), which selects both the front and the rear
 * adaptive damper parts. Run with
 * {@code BEAMCRAFT_JBEAM_CORPUS=<...>/content/vehicles}.
 */
class AdaptiveDamperEtkCorpusTest {
    private static final String REAR = "adaptiveRearDamper";
    private static final String FRONT = "adaptiveFrontDamper";

    @Test
    void stockTtSportPlusRegistersBothAdaptiveDamperControllers() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(corpusRoot, "etk800", "846x_ttsport_plus_DCT.pc", registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("etk800", config, registry, vehicle));

        assertEquals(List.of(FRONT, REAR), vehicle.adaptiveDampers.controllerNames().stream().sorted().toList());
        for (String controller : List.of(FRONT, REAR)) {
            AdaptiveDamperController instance = vehicle.adaptiveDampers.controller(controller);
            assertNotNull(instance);
            assertEquals(2, instance.beamIndices().length,
                    controller + " must drive exactly the two shock beams of its axle");
            assertEquals("regular", instance.currentModeName(),
                    "the authored regular mode is the assembly default");
            assertEquals(List.of("soft", "regular", "hard").stream().sorted().toList(),
                    instance.modes().keySet().stream().sorted().toList());
        }

        // Every named shock beam is a bounded beam.
        assertTrue(vehicle.boundedBeams.count > 0);
        assertEquals(0, vehicle.boundedBeams.indicesForName("noSuchBeam").length);
    }

    @Test
    void modeSwitchesStayBoundedAndNeverCompoundOnAStockVehicle() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(corpusRoot, "etk800", "846x_ttsport_plus_DCT.pc", registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("etk800", config, registry, vehicle));

        BoundedBeamContainer beams = vehicle.boundedBeams;
        int rearBeam = vehicle.adaptiveDampers.controller(REAR).beamIndices()[0];
        float ceiling = beams.dampStabilityCeiling[rearBeam];

        assertFalse(vehicle.adaptiveDampers.applyDamperMode(REAR, "missing"));
        assertTrue(vehicle.adaptiveDampers.setDamperMode(REAR, "soft"));
        vehicle.solveInternalForces(1.0f / PhysicsWorld.invPhysicsDT, 1.0f);
        float soft = beams.dampRebound[rearBeam];
        assertTrue(soft <= ceiling * (1.0f + 1.0e-6f), "soft must respect the stability ceiling");

        assertTrue(vehicle.adaptiveDampers.setDamperMode(REAR, "hard"));
        vehicle.solveInternalForces(1.0f / PhysicsWorld.invPhysicsDT, 1.0f);
        float hard = beams.dampRebound[rearBeam];
        assertTrue(hard <= ceiling * (1.0f + 1.0e-6f), "hard must respect the stability ceiling");
        assertTrue(hard >= soft, "the hard mode must not damp less than soft");

        // Round-tripping must land back on exactly the same coefficients.
        for (int i = 0; i < 10; i++) {
            vehicle.adaptiveDampers.applyDamperMode(REAR, "hard");
            vehicle.adaptiveDampers.applyDamperMode(REAR, "soft");
        }
        assertEquals(soft, beams.dampRebound[rearBeam], 1.0e-3f,
                "repeated switching must not compound the coefficients");

        vehicle.reset();
        assertEquals("soft", vehicle.adaptiveDampers.currentMode(REAR));
        assertEquals(soft, beams.dampRebound[rearBeam], 1.0e-3f,
                "reset must reapply the selected mode from the authored base");
    }
}
