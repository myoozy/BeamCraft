package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Coverage for {@code dampCutoffHz}, the BeamNG beam property that low-passes the
 * axial relative velocity feeding the damping force.
 *
 * <p>The implementation is a causal one-pole filter with persistent per-beam
 * state: {@code y += alpha (x - y)} with {@code alpha = 1 - exp(-2 pi fc dt)}.
 * An unspecified or non-positive cutoff bypasses the filter entirely, so the raw
 * relative velocity reaches the damping coefficient exactly as before.
 *
 * <p>Every vehicle below uses massless nodes, a zero spring and a beam whose rest
 * length equals its current separation. The solve therefore leaves geometry and
 * velocities untouched and {@code nodes.forceX[0]} is exactly
 * {@code damp * (damping velocity)}, which makes the filter directly observable.
 */
class BeamDampCutoffTest {

    private static final float DT = 1.0f / PhysicsWorld.invPhysicsDT;
    private static final float DAMP = 1000.0f;
    private static final float REST = 1.0f;

    /** Cutoff low enough that the 2 kHz sub-step is far above it. */
    private static final float CUTOFF_HZ = 5.0f;

    // --- disabled identity -------------------------------------------------

    @Test
    void unspecifiedAndNonPositiveCutoffKeepRawRelativeVelocityDamping() {
        for (float cutoff : new float[]{-1.0f, 0.0f, Float.NaN}) {
            assertEquals(DAMP * 3.0f, firstStepForce(normalBeam(cutoff), 3.0f), 1.0e-3f,
                    "cutoff " + cutoff + " must leave damping unfiltered");
        }
    }

    @Test
    void disabledCutoffTracksEverySubstepExactly() {
        SoftBodyVehicle vehicle = normalBeam(0.0f);
        for (int step = 0; step < 50; step++) {
            float velocity = (step % 2 == 0) ? 2.5f : -2.5f;
            vehicle.nodes.velX[1] = velocity;
            vehicle.solveInternalForces(DT, 1.0f);
            assertEquals(DAMP * velocity, vehicle.nodes.forceX[0], 1.0e-2f,
                    "an unfiltered beam must not lag or attenuate");
        }
    }

    // --- filter response ---------------------------------------------------

    @Test
    void firstSubstepUsesTheCutoffCoefficient() {
        float alpha = BeamContainer.dampingCutoffAlpha(CUTOFF_HZ, DT);
        assertTrue(alpha > 0.0f && alpha < 1.0f, "alpha must be a valid one-pole coefficient");

        assertEquals(DAMP * alpha * 3.0f, firstStepForce(normalBeam(CUTOFF_HZ), 3.0f), 1.0e-3f,
                "the first sub-step smooths the raw velocity by alpha");
    }

    @Test
    void aboveCutoffAlternatingVelocityIsAttenuatedToTheNyquistGain() {
        float alpha = BeamContainer.dampingCutoffAlpha(CUTOFF_HZ, DT);
        float expectedGain = alpha / (2.0f - alpha);

        float observed = alternatingSteadyStateForce(normalBeam(CUTOFF_HZ), 1.0f, 6_000) / DAMP;

        assertTrue(observed < 0.05f, "a 5 Hz cutoff must strongly attenuate the 1 kHz alternation");
        assertEquals(expectedGain, observed, 1.0e-3f,
                "the alternating steady-state gain is alpha / (2 - alpha)");
    }

    @Test
    void lowFrequencyAndDcVelocityPassThroughUnattenuated() {
        SoftBodyVehicle vehicle = normalBeam(CUTOFF_HZ);
        vehicle.nodes.velX[1] = 4.0f;
        for (int step = 0; step < 2_000; step++) {
            vehicle.solveInternalForces(DT, 1.0f);
        }
        assertEquals(DAMP * 4.0f, vehicle.nodes.forceX[0], 1.0e-2f,
                "a constant (DC) velocity must reach unity gain once the transient decays");
    }

    @Test
    void coefficientIsTimestepConsistent() {
        // Two dt steps of the exact one-pole solution equal one 2 dt step.
        float oneStep = BeamContainer.dampingCutoffAlpha(CUTOFF_HZ, 2.0f * DT);
        float half = BeamContainer.dampingCutoffAlpha(CUTOFF_HZ, DT);
        assertEquals(oneStep, 1.0f - (1.0f - half) * (1.0f - half), 1.0e-6f);

        SoftBodyVehicle longStep = normalBeam(CUTOFF_HZ);
        longStep.nodes.velX[1] = 3.0f;
        longStep.solveInternalForces(2.0f * DT, 1.0f);

        SoftBodyVehicle shortSteps = normalBeam(CUTOFF_HZ);
        shortSteps.nodes.velX[1] = 3.0f;
        shortSteps.solveInternalForces(DT, 1.0f);
        shortSteps.solveInternalForces(DT, 1.0f);

        assertEquals(longStep.nodes.forceX[0], shortSteps.nodes.forceX[0], 1.0e-3f,
                "the filter response must be independent of the sub-step split");
    }

    // --- reset -------------------------------------------------------------

    @Test
    void resetClearsFilterStateWithoutAStartupImpulse() {
        SoftBodyVehicle vehicle = normalBeam(CUTOFF_HZ);
        vehicle.nodes.velX[1] = 5.0f;
        for (int step = 0; step < 200; step++) {
            vehicle.solveInternalForces(DT, 1.0f);
        }
        assertTrue(vehicle.normalBeams.dampFilterState[0] > 0.0f, "state must have accumulated");

        vehicle.normalBeams.reset();
        assertEquals(0.0f, vehicle.normalBeams.dampFilterState[0], 0.0f,
                "reset must clear the persistent filter state");

        vehicle.nodes.velX[1] = 0.0f;
        vehicle.solveInternalForces(DT, 1.0f);
        assertEquals(0.0f, vehicle.nodes.forceX[0], 1.0e-6f,
                "a cleared filter at rest must not inject a force");

        // The first post-reset step with motion starts the ramp, it never spikes
        // past the unfiltered damping force.
        vehicle.nodes.velX[1] = 5.0f;
        vehicle.solveInternalForces(DT, 1.0f);
        float firstStep = vehicle.nodes.forceX[0];
        assertTrue(firstStep > 0.0f && firstStep < DAMP * 5.0f,
                "restarting the filter must ramp up, not spike");
    }

    @Test
    void vehicleResetAlsoClearsTheFilterState() {
        SoftBodyVehicle vehicle = normalBeam(CUTOFF_HZ);
        vehicle.nodes.velX[1] = 5.0f;
        vehicle.solveInternalForces(DT, 1.0f);
        assertTrue(vehicle.normalBeams.dampFilterState[0] > 0.0f);

        vehicle.reset();
        assertEquals(0.0f, vehicle.normalBeams.dampFilterState[0], 0.0f);
        assertEquals(0.0f, vehicle.nodes.forceX[0], 0.0f);
    }

    // --- applicability -----------------------------------------------------

    @Test
    void boundedBeamsFilterTheirDampingVelocity() {
        float gang = alternatingSteadyStateForce(boundedBeam(CUTOFF_HZ), 1.0f, 6_000);
        float plain = alternatingSteadyStateForce(boundedBeam(-1.0f), 1.0f, 6_000);
        assertEquals(DAMP, plain, 1.0e-1f, "without a cutoff the bounded beam is unattenuated");
        assertTrue(gang < 0.05f * DAMP, "bounded damping must be attenuated by the cutoff");
    }

    @Test
    void lBeamsFilterTheirDampingVelocity() {
        float gang = alternatingSteadyStateForce(lBeam(CUTOFF_HZ), 2.0f, 6_000);
        float plain = alternatingSteadyStateForce(lBeam(-1.0f), 2.0f, 6_000);
        assertEquals(DAMP, plain, 1.0e-1f,
                "an L-beam with a 2 m/s lateral node velocity damps at exactly 1 m/s");
        assertTrue(gang < 0.05f * DAMP, "L-beam damping must be attenuated by the cutoff");
    }

    @Test
    void supportAndAnisotropicBeamsIgnoreTheCutoff() {
        // BeamNG documents dampCutoffHz for normal, bounded and L-beams only, so
        // the other families must keep raw relative-velocity damping.
        for (int type : new int[]{BeamContainer.BEAM_SUPPORT, BeamContainer.BEAM_ANISOTROPIC}) {
            SoftBodyVehicle vehicle = twoNodeBeam(TestBeamBuilder.normal().type(type), CUTOFF_HZ, REST);
            assertEquals(DAMP, alternatingSteadyStateForce(vehicle, 1.0f, 500), 1.0e-1f,
                    "beam type " + type + " must not filter its damping velocity");
        }
    }

    // --- stability integration --------------------------------------------

    @Test
    void cutoffRaisesTheAuthoredDampingCeiling() {
        // reducedMass(1,1) * 2000 * 0.95 == 950 for the unfiltered budget.
        SoftBodyVehicle unfiltered = stabilityVehicle(-1.0f, 5_000.0f);
        assertEquals(950.0f, unfiltered.normalBeams.damp[0], 1.0f,
                "without a cutoff the authored damping is clamped to the safety budget");

        SoftBodyVehicle filtered = stabilityVehicle(CUTOFF_HZ, 5_000.0f);
        assertEquals(5_000.0f, filtered.normalBeams.damp[0], 1.0f,
                "a cutoff attenuates the high-frequency damping, so the authored value survives");
    }

    @Test
    void cutoffKeepsADefensibleSafetyCeiling() {
        SoftBodyVehicle vehicle = stabilityVehicle(CUTOFF_HZ, 1.0e9f);
        float hfGain = vehicle.normalBeams.cutoffHighFrequencyGain(0, DT);
        float expectedCeiling = 950.0f / hfGain;

        assertEquals(expectedCeiling, vehicle.normalBeams.damp[0],
                0.01f * expectedCeiling, "the ceiling scales by 1 / hfGain");
        assertTrue(vehicle.normalBeams.damp[0] < 1.0e9f,
                "a pathological authored damping must still be bounded");
    }

    @Test
    void cutoffHighFrequencyGainIsOneWhenDisabled() {
        SoftBodyVehicle vehicle = stabilityVehicle(-1.0f, 100.0f);
        assertEquals(1.0f, vehicle.normalBeams.cutoffHighFrequencyGain(0, DT), 0.0f);
    }

    @Test
    void filteredDampingStaysStableAtTheRaisedCeiling() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.massNode("a", 0.0f, 0.0f, 0.0f, 1.0f));
        vehicle.addNode(TestBeamBuilder.massNode("b", 1.0f, 0.0f, 0.0f, 1.0f));
        // A ~1 Hz spring mode sits below the cutoff, so the damping is not filtered away.
        vehicle.addBeam(TestBeamBuilder.normal().spring(20.0f).damp(0.0f).dampCutoffHz(CUTOFF_HZ).build());

        // The largest damping the raised ceiling admits for the 950 budget below.
        float authored = 950.0f / vehicle.normalBeams.cutoffHighFrequencyGain(0, DT);
        vehicle.normalBeams.damp[0] = authored;
        vehicle.finalizePhysicsSetup();

        assertEquals(authored, vehicle.normalBeams.damp[0], 0.01f * authored,
                "the raised ceiling must admit the authored value instead of 950");
        assertTrue(authored > 950.0f, "this test is only meaningful above the unfiltered budget");

        // The extreme authoring must still ring down instead of diverging.
        vehicle.nodes.posX[1] = 1.2f;
        vehicle.nodes.velX[1] = 10.0f;
        for (int step = 0; step < 20_000; step++) {
            vehicle.solveInternalForces(DT, 1.0f);
            assertTrue(Float.isFinite(vehicle.nodes.velX[0]) && Float.isFinite(vehicle.nodes.velX[1]),
                    "the filtered damping must not destabilise the sub-step");
        }
        assertTrue(Math.abs(vehicle.nodes.velX[1] - vehicle.nodes.velX[0]) < 1.0f,
                "the relative velocity must decay, not grow");
    }

    // --- helpers -----------------------------------------------------------

    private static float firstStepForce(SoftBodyVehicle vehicle, float velocity) {
        vehicle.nodes.velX[1] = velocity;
        vehicle.solveInternalForces(DT, 1.0f);
        return vehicle.nodes.forceX[0];
    }

    /**
     * Drives an alternating axial velocity for {@code steps} sub-steps and returns
     * the last damping force normalised to a lengthening (positive) beam, so the
     * steady-state magnitude is directly comparable to the sub-step inputs.
     */
    private static float alternatingSteadyStateForce(SoftBodyVehicle vehicle, float amplitude, int steps) {
        float lastSign = 1.0f;
        for (int step = 0; step < steps; step++) {
            lastSign = (step % 2 == 0) ? 1.0f : -1.0f;
            vehicle.nodes.velX[1] = lastSign * amplitude;
            vehicle.solveInternalForces(DT, 1.0f);
        }
        return vehicle.nodes.forceX[0] * lastSign;
    }

    private static SoftBodyVehicle normalBeam(float cutoffHz) {
        return twoNodeBeam(TestBeamBuilder.normal(), cutoffHz, REST);
    }

    private static SoftBodyVehicle boundedBeam(float cutoffHz) {
        return twoNodeBeam(TestBeamBuilder.bounded(), cutoffHz, REST);
    }

    /**
     * Three massless nodes, node 2 free to slide along +x. Driving node 2 at
     * 2 m/s makes the L-beam's target-relative damping velocity exactly 1 m/s.
     */
    private static SoftBodyVehicle lBeam(float cutoffHz) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.masslessNode("a", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("b", 1.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("c", 0.0f, 1.0f, 0.0f));
        vehicle.addBeam(TestBeamBuilder.lBeam("c").damp(DAMP).dampCutoffHz(cutoffHz).build());
        return vehicle;
    }

    /** Two unit-mass nodes so the unfiltered damping budget is exactly 950. */
    private static SoftBodyVehicle stabilityVehicle(float cutoffHz, float damp) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.massNode("a", 0.0f, 0.0f, 0.0f, 1.0f));
        vehicle.addNode(TestBeamBuilder.massNode("b", 1.0f, 0.0f, 0.0f, 1.0f));
        vehicle.addBeam(TestBeamBuilder.normal().damp(damp).dampCutoffHz(cutoffHz).build());
        vehicle.finalizePhysicsSetup();
        return vehicle;
    }

    private static SoftBodyVehicle twoNodeBeam(TestBeamBuilder builder, float cutoffHz, float rest) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.masslessNode("a", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("b", rest, 0.0f, 0.0f));
        vehicle.addBeam(builder.damp(DAMP).dampCutoffHz(cutoffHz).build());
        return vehicle;
    }
}
