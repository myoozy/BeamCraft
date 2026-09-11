package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Behaviour of the standalone adaptive damper actuator backend.
 *
 * <p>The fixture beams reproduce the stock ETK800 rear adaptive shock
 * ({@code beamDamp 6000}, {@code beamDampFast 1500}, {@code beamDampRebound 8500},
 * {@code beamDampReboundFast 2833}, {@code beamDampVelocitySplit 0.25},
 * {@code dampCutoffHz 500}) with the stock mode table
 * ({@code soft 0.7/1/0.65/1/0.7}, {@code hard 1.5/1/1.4/1/1.2}), and the front
 * table ({@code hard 1.35/1/1.26/1/1.2}) for the multi-controller cases.
 */
class AdaptiveDamperActuatorsTest {

    private static final float DT = 1.0f / PhysicsWorld.invPhysicsDT;
    private static final float TOL = 1.0e-2f;

    private static final float AUTHORED_DAMP = 6000.0f;
    private static final float AUTHORED_FAST = 1500.0f;
    private static final float AUTHORED_REBOUND = 8500.0f;
    private static final float AUTHORED_REBOUND_FAST = 2833.0f;
    private static final float AUTHORED_SPLIT = 0.25f;

    private static final float CUTOFF_HZ = 500.0f;
    /** Heavy nodes keep the authored ETK damping comfortably inside the stability budget. */
    private static final float HEAVY_MASS = 10.0f;

    private static final String CONTROLLER = "adaptiveRearDamper";
    private static final String FRONT_CONTROLLER = "adaptiveFrontDamper";

    // --- BeamSpec name preservation + lookup -------------------------------

    @Test
    void beamNamesSurviveTheSpecAndBuildAOneToManyLookup() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));

        assertArrayEquals(new int[]{0}, vehicle.boundedBeams.indicesForName("shock_RR"));
        assertArrayEquals(new int[]{1}, vehicle.boundedBeams.indicesForName("shock_RL"));
        assertArrayEquals(new int[0], vehicle.boundedBeams.indicesForName("shock_XX"));
        assertArrayEquals(new int[0], vehicle.boundedBeams.indicesForName(null));
    }

    @Test
    void duplicateBeamNamesTargetEveryMatchingBoundedBeam() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RR"), List.of(rearSpecWithNames(List.of("shock_RR"))));

        assertArrayEquals(new int[]{0, 1}, vehicle.boundedBeams.indicesForName("shock_RR"));
        assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard"));

        for (int i = 0; i < 2; i++) {
            assertEquals(AUTHORED_REBOUND * 1.4f, vehicle.boundedBeams.dampRebound[i], TOL,
                    "every bounded beam sharing the name must be driven");
        }
    }

    // --- exact ETK-style multipliers ---------------------------------------

    @Test
    void softRegularAndHardApplyTheExactEtkMultipliers() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));

        assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "soft"));
        assertChannels(vehicle, 0,
                AUTHORED_DAMP * 0.7f, AUTHORED_FAST, AUTHORED_REBOUND * 0.65f,
                AUTHORED_REBOUND_FAST, AUTHORED_SPLIT * 0.7f);

        assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "regular"));
        assertChannels(vehicle, 0,
                AUTHORED_DAMP, AUTHORED_FAST, AUTHORED_REBOUND, AUTHORED_REBOUND_FAST, AUTHORED_SPLIT);

        assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard"));
        assertChannels(vehicle, 0,
                AUTHORED_DAMP * 1.5f, AUTHORED_FAST, AUTHORED_REBOUND * 1.4f,
                AUTHORED_REBOUND_FAST, AUTHORED_SPLIT * 1.2f);
    }

    @Test
    void theScaledVelocitySplitIsPassedToBothCompressionAndRebound() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        assertEquals(AUTHORED_SPLIT, vehicle.boundedBeams.dampVelocitySplitRebound[0], 0.0f,
                "an unauthored rebound split falls back to the common one");

        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        assertEquals(AUTHORED_SPLIT * 1.2f, vehicle.boundedBeams.dampVelocitySplit[0], TOL);
        assertEquals(AUTHORED_SPLIT * 1.2f, vehicle.boundedBeams.dampVelocitySplitRebound[0], TOL,
                "adaptiveDampers.lua passes the same scaled split to both channels");
    }

    @Test
    void everyBeamOfTheControllerIsDriven() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        for (int i = 0; i < 2; i++) {
            assertEquals(AUTHORED_REBOUND * 1.4f, vehicle.boundedBeams.dampRebound[i], TOL);
        }
    }

    // --- no compounding ----------------------------------------------------

    @Test
    void repeatedSwitchesNeverCompound() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));

        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        float firstHard = vehicle.boundedBeams.damp[0];
        float firstHardRebound = vehicle.boundedBeams.dampRebound[0];
        for (int i = 0; i < 20; i++) {
            assertTrue(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard"));
        }
        assertEquals(firstHard, vehicle.boundedBeams.damp[0], 0.0f,
                "re-selecting the same mode must be idempotent");
        assertEquals(firstHardRebound, vehicle.boundedBeams.dampRebound[0], 0.0f);

        // hard -> soft -> hard returns to exactly the first hard values, so the
        // authored base was never consumed by the intermediate soft step.
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "soft");
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        assertEquals(firstHard, vehicle.boundedBeams.damp[0], TOL);
        assertEquals(firstHardRebound, vehicle.boundedBeams.dampRebound[0], TOL);
    }

    @Test
    void aHardToSoftSwitchRestoresTheScaledAuthoredBaseNotARescaledOne() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        for (int i = 0; i < 10; i++) {
            vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
            vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "soft");
        }
        assertChannels(vehicle, 0,
                AUTHORED_DAMP * 0.7f, AUTHORED_FAST, AUTHORED_REBOUND * 0.65f,
                AUTHORED_REBOUND_FAST, AUTHORED_SPLIT * 0.7f);

        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        assertChannels(vehicle, 0,
                AUTHORED_DAMP * 1.5f, AUTHORED_FAST, AUTHORED_REBOUND * 1.4f,
                AUTHORED_REBOUND_FAST, AUTHORED_SPLIT * 1.2f);
    }

    // --- unknown inputs ----------------------------------------------------

    @Test
    void unknownControllerOrModeIsRejectedWithoutMutatingState() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        float damp = vehicle.boundedBeams.damp[0];
        float rebound = vehicle.boundedBeams.dampRebound[0];
        float split = vehicle.boundedBeams.dampVelocitySplit[0];
        String mode = vehicle.adaptiveDampers.currentMode(CONTROLLER);

        assertFalse(vehicle.adaptiveDampers.applyDamperMode("noSuchController", "hard"));
        assertFalse(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "noSuchMode"));
        assertFalse(vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, null));
        assertFalse(vehicle.adaptiveDampers.applyDamperMode(null, "hard"));
        assertFalse(vehicle.adaptiveDampers.applyDamperMode(null, null));

        assertFalse(vehicle.adaptiveDampers.setDamperMode("noSuchController", "hard"));
        assertFalse(vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "noSuchMode"));
        assertFalse(vehicle.adaptiveDampers.setDamperMode(null, null));
        assertEquals(0, vehicle.adaptiveDampers.setAllDamperModes("noSuchMode"));

        // A rejected queued command must not be applied by the next sub-step either.
        vehicle.solveInternalForces(DT, 1.0f);

        assertEquals(damp, vehicle.boundedBeams.damp[0], 0.0f);
        assertEquals(rebound, vehicle.boundedBeams.dampRebound[0], 0.0f);
        assertEquals(split, vehicle.boundedBeams.dampVelocitySplit[0], 0.0f);
        assertEquals(mode, vehicle.adaptiveDampers.currentMode(CONTROLLER));
    }

    @Test
    void controllerQueriesAreSafeForUnknownNames() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        assertTrue(vehicle.adaptiveDampers.hasController(CONTROLLER));
        assertFalse(vehicle.adaptiveDampers.hasController("nope"));
        assertNull(vehicle.adaptiveDampers.controller("nope"));
        assertNull(vehicle.adaptiveDampers.currentMode("nope"));
        assertNotNull(vehicle.adaptiveDampers.controller(CONTROLLER));
    }

    // --- default / init / reset -------------------------------------------

    @Test
    void regularIsAppliedAtAssemblyWhenAuthored() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        assertEquals("regular", vehicle.adaptiveDampers.currentMode(CONTROLLER));
        assertChannels(vehicle, 0,
                AUTHORED_DAMP, AUTHORED_FAST, AUTHORED_REBOUND, AUTHORED_REBOUND_FAST, AUTHORED_SPLIT);
    }

    @Test
    void withoutARegularModeTheAuthoredCoefficientsAreKept() {
        Map<String, AdaptiveDamperMode> modes = new LinkedHashMap<>();
        modes.put("soft", new AdaptiveDamperMode("soft", 0.7f, 1f, 0.65f, 1f, 0.7f));
        modes.put("hard", new AdaptiveDamperMode("hard", 1.5f, 1f, 1.4f, 1f, 1.2f));
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"),
                List.of(new AdaptiveDamperSpec(CONTROLLER, List.of("shock_RR", "shock_RL"), modes)));

        assertNull(vehicle.adaptiveDampers.currentMode(CONTROLLER));
        assertChannels(vehicle, 0,
                AUTHORED_DAMP, AUTHORED_FAST, AUTHORED_REBOUND, AUTHORED_REBOUND_FAST, AUTHORED_SPLIT);
    }

    @Test
    void resetReappliesTheSelectedModeWithoutCumulativeScaling() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        float hardDamp = vehicle.boundedBeams.damp[0];
        float hardRebound = vehicle.boundedBeams.dampRebound[0];

        for (int i = 0; i < 5; i++) {
            vehicle.reset();
            assertEquals("hard", vehicle.adaptiveDampers.currentMode(CONTROLLER),
                    "reset must keep the selected mode");
            assertEquals(hardDamp, vehicle.boundedBeams.damp[0], TOL);
            assertEquals(hardRebound, vehicle.boundedBeams.dampRebound[0], TOL);
        }

        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "soft");
        vehicle.reset();
        assertChannels(vehicle, 0,
                AUTHORED_DAMP * 0.7f, AUTHORED_FAST, AUTHORED_REBOUND * 0.65f,
                AUTHORED_REBOUND_FAST, AUTHORED_SPLIT * 0.7f);
    }

    @Test
    void clearingTheVehicleDropsControllersAndPendingCommands() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        assertTrue(vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "hard"));

        vehicle.clear();

        assertFalse(vehicle.adaptiveDampers.hasController(CONTROLLER));
        assertTrue(vehicle.adaptiveDampers.controllerNames().isEmpty());
    }

    // --- thread boundary ---------------------------------------------------

    @Test
    void queuedCommandIsAppliedAtTheNextSubstepNotBefore() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));

        assertTrue(vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "hard"));
        assertEquals(AUTHORED_REBOUND, vehicle.boundedBeams.dampRebound[0], 0.0f,
                "a queued command must not touch the SoA arrays off the sub-step thread");
        assertEquals("regular", vehicle.adaptiveDampers.currentMode(CONTROLLER));

        vehicle.solveInternalForces(DT, 1.0f);

        assertEquals(AUTHORED_REBOUND * 1.4f, vehicle.boundedBeams.dampRebound[0], TOL);
        assertEquals("hard", vehicle.adaptiveDampers.currentMode(CONTROLLER));
    }

    @Test
    void theLastQueuedCommandForAControllerWins() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "soft");
        vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "hard");
        vehicle.adaptiveDampers.setDamperMode(CONTROLLER, "regular");

        vehicle.solveInternalForces(DT, 1.0f);

        assertEquals("regular", vehicle.adaptiveDampers.currentMode(CONTROLLER));
        assertEquals(AUTHORED_REBOUND, vehicle.boundedBeams.dampRebound[0], TOL);
    }

    @Test
    void setAllDamperModesReachesEveryControllerSharingTheModeName() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL", "shock_FR", "shock_FL"),
                List.of(rearSpec(), frontSpec()));

        assertEquals(2, vehicle.adaptiveDampers.setAllDamperModes("hard"));
        vehicle.solveInternalForces(DT, 1.0f);

        assertEquals("hard", vehicle.adaptiveDampers.currentMode(CONTROLLER));
        assertEquals("hard", vehicle.adaptiveDampers.currentMode(FRONT_CONTROLLER));
        // The rear table scales the base channel by 1.5, the front table by 1.35.
        assertEquals(AUTHORED_DAMP * 1.5f, vehicle.boundedBeams.damp[0], TOL);
        assertEquals(AUTHORED_DAMP * 1.35f, vehicle.boundedBeams.damp[2], TOL);
        assertEquals(0, vehicle.adaptiveDampers.setAllDamperModes("noSuchMode"));
    }

    // --- stability ceiling interaction ------------------------------------

    @Test
    void aModeMayRaiseDampingAboveTheAuthoredValueUpToTheBudgetCeiling() {
        SoftBodyVehicle vehicle = ceilingVehicle(1.0f, 1000.0f);
        float ceiling = vehicle.boundedBeams.dampStabilityCeiling[0];

        assertEquals(950.0f / vehicle.boundedBeams.cutoffHighFrequencyGain(0, DT), ceiling,
                0.01f * ceiling, "the stored ceiling is the cutoff-aware budget for this beam");
        assertTrue(ceiling > 1000.0f, "this test is only meaningful above the authored value");
        assertEquals(1000.0f, vehicle.boundedBeams.dampRebound[0], TOL,
                "assembly keeps the authored damping, it is inside the budget");

        // hard scales the authored rebound by 1.4 -> above authored, still under budget.
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        assertEquals(1400.0f, vehicle.boundedBeams.dampRebound[0], TOL);

        // A wildly scaled mode is clamped to the ceiling instead of destabilising.
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "extreme");
        assertEquals(ceiling, vehicle.boundedBeams.dampRebound[0], TOL);

        // ...and coming back down still starts from the authored base.
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "regular");
        assertEquals(1000.0f, vehicle.boundedBeams.dampRebound[0], TOL);
    }

    @Test
    void assemblyClampsAnOversizedAuthoredValueButKeepsItAsTheBase() {
        SoftBodyVehicle vehicle = ceilingVehicle(1.0f, 1.0e9f);
        float ceiling = vehicle.boundedBeams.dampStabilityCeiling[0];

        assertEquals(ceiling, vehicle.boundedBeams.dampRebound[0], 0.01f * ceiling,
                "the oversized authored value is clamped to the budget at assembly");
        assertEquals(1.0e9f, vehicle.boundedBeams.authoredDampRebound[0], 0.0f,
                "the authored value itself is preserved verbatim");

        // Every mode stays inside the ceiling and the authored base is never lost.
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "soft");
        assertTrue(vehicle.boundedBeams.dampRebound[0] <= ceiling * (1.0f + 1.0e-6f));
        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");
        assertEquals(ceiling, vehicle.boundedBeams.dampRebound[0], 0.01f * ceiling);
        assertEquals(1.0e9f, vehicle.boundedBeams.authoredDampRebound[0], 0.0f);
    }

    @Test
    void modeChangesLeaveUnrelatedBoundedBeamsUntouched() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL", "unrelated"), List.of(rearSpec()));

        int beamCount = vehicle.boundedBeams.count;
        int[] node1 = vehicle.boundedBeams.node1.clone();
        float unrelatedDamp = vehicle.boundedBeams.damp[2];
        float unrelatedRebound = vehicle.boundedBeams.dampRebound[2];
        float unrelatedSpring = vehicle.boundedBeams.spring[2];

        vehicle.adaptiveDampers.applyDamperMode(CONTROLLER, "hard");

        assertEquals(beamCount, vehicle.boundedBeams.count, "no beam may be added or removed");
        assertArrayEquals(node1, vehicle.boundedBeams.node1, "beam topology must not be rebuilt");
        assertEquals(unrelatedDamp, vehicle.boundedBeams.damp[2], 0.0f);
        assertEquals(unrelatedRebound, vehicle.boundedBeams.dampRebound[2], 0.0f);
        assertEquals(unrelatedSpring, vehicle.boundedBeams.spring[2], 0.0f);
    }

    @Test
    void aFixtureSanityCheckConfirmsTheCeilingIsAboveTheAuthoredValues() {
        SoftBodyVehicle vehicle = shockVehicle(HEAVY_MASS, CUTOFF_HZ,
                List.of("shock_RR", "shock_RL"), List.of(rearSpec()));
        assertTrue(vehicle.boundedBeams.dampStabilityCeiling[0] > AUTHORED_REBOUND,
                "the multiplier tests only exercise scaling while the authored value fits the budget");
    }

    // --- helpers -----------------------------------------------------------

    private static void assertChannels(SoftBodyVehicle vehicle, int beam,
                                       float damp, float dampFast, float dampRebound,
                                       float dampReboundFast, float split) {
        BoundedBeamContainer beams = vehicle.boundedBeams;
        assertEquals(damp, beams.damp[beam], TOL, "beamDamp");
        assertEquals(dampFast, beams.dampFast[beam], TOL, "beamDampFast");
        assertEquals(dampRebound, beams.dampRebound[beam], TOL, "beamDampRebound");
        assertEquals(dampReboundFast, beams.dampReboundFast[beam], TOL, "beamDampReboundFast");
        assertEquals(split, beams.dampVelocitySplit[beam], TOL, "beamDampVelocitySplit");
        assertEquals(split, beams.dampVelocitySplitRebound[beam], TOL, "beamDampVelocitySplitRebound");
    }

    /** Stock ETK800 rear shock beam, produced by the real parser from the real authoring values. */
    private static TestBeamBuilder shock(String name, String node1, String node2, float dampCutoffHz) {
        return TestBeamBuilder.bounded()
                .between(node1, node2)
                .name(name)
                .spring(0.0f)
                .damp(AUTHORED_DAMP)
                .dampChannels(AUTHORED_FAST, AUTHORED_REBOUND, AUTHORED_REBOUND_FAST)
                .dampSplits(AUTHORED_SPLIT, -1.0f)
                .dampCutoffHz(dampCutoffHz);
    }

    /**
     * One isolated unit-mass beam pair per name, so every beam's damping budget is
     * independent of the others and no stability scaling is triggered.
     */
    private static SoftBodyVehicle shockVehicle(float mass, float dampCutoffHz,
                                                List<String> beamNames,
                                                List<AdaptiveDamperSpec> specs) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        for (int i = 0; i < beamNames.size(); i++) {
            String node1 = "n" + (2 * i);
            String node2 = "n" + (2 * i + 1);
            vehicle.addNode(TestBeamBuilder.massNode(node1, 2.0f * i, 0.0f, 0.0f, mass));
            vehicle.addNode(TestBeamBuilder.massNode(node2, 2.0f * i + 1.0f, 0.0f, 0.0f, mass));
            vehicle.addBeam(shock(beamNames.get(i), node1, node2, dampCutoffHz).build());
        }
        vehicle.setAdaptiveDamperSpecs(specs);
        vehicle.finalizePhysicsSetup();
        return vehicle;
    }

    /** A single unit-mass beam with a modest authored rebound, for ceiling coverage. */
    private static SoftBodyVehicle ceilingVehicle(float mass, float authoredRebound) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.massNode("a", 0.0f, 0.0f, 0.0f, mass));
        vehicle.addNode(TestBeamBuilder.massNode("b", 1.0f, 0.0f, 0.0f, mass));
        vehicle.addBeam(TestBeamBuilder.bounded()
                .between("a", "b")
                .name("shock_RR")
                .damp(0.0f)
                .dampChannels(0.0f, authoredRebound, 0.0f)
                .dampSplits(AUTHORED_SPLIT, -1.0f)
                .dampCutoffHz(CUTOFF_HZ)
                .build());
        vehicle.setAdaptiveDamperSpecs(List.of(rearSpecWithNames(List.of("shock_RR"))));
        vehicle.finalizePhysicsSetup();
        return vehicle;
    }

    /** Stock ETK800 rear mode table plus an out-of-range mode for ceiling coverage. */
    private static AdaptiveDamperSpec rearSpec() {
        Map<String, AdaptiveDamperMode> modes = new LinkedHashMap<>();
        modes.put("soft", new AdaptiveDamperMode("soft", 0.7f, 1f, 0.65f, 1f, 0.7f));
        modes.put("regular", new AdaptiveDamperMode("regular", 1f, 1f, 1f, 1f, 1f));
        modes.put("hard", new AdaptiveDamperMode("hard", 1.5f, 1f, 1.4f, 1f, 1.2f));
        modes.put("extreme", new AdaptiveDamperMode("extreme", 500f, 1f, 1.0e6f, 1f, 1f));
        return new AdaptiveDamperSpec(CONTROLLER, List.of("shock_RR", "shock_RL"), modes);
    }

    private static AdaptiveDamperSpec rearSpecWithNames(List<String> names) {
        AdaptiveDamperSpec base = rearSpec();
        return new AdaptiveDamperSpec(base.instanceName(), names, base.modes());
    }

    /** Stock ETK800 front mode table. */
    private static AdaptiveDamperSpec frontSpec() {
        Map<String, AdaptiveDamperMode> modes = new LinkedHashMap<>();
        modes.put("soft", new AdaptiveDamperMode("soft", 0.65f, 1f, 0.6f, 1f, 0.7f));
        modes.put("regular", new AdaptiveDamperMode("regular", 1f, 1f, 1f, 1f, 1f));
        modes.put("hard", new AdaptiveDamperMode("hard", 1.35f, 1f, 1.26f, 1f, 1.2f));
        return new AdaptiveDamperSpec(FRONT_CONTROLLER, List.of("shock_FR", "shock_FL"), modes);
    }
}
