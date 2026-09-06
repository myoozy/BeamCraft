package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Regression coverage for the plastic-deformation / hardening path of a normal
 * beam. The internal-force solve now runs through
 * {@link VehicleInternalForceSolver}; these assertions pin down that an
 * over-yield load permanently lengthens the rest state and hardens the yield
 * surface without breaking the beam.
 */
class BeamPlasticDeformationTest {
    @Test
    void overYieldStretchPermanentlyLengthensRestAndHardens() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.nodes.count = 2;
        // Massless nodes: keep geometry fixed so only the beam plastic update runs.
        vehicle.nodes.posX[0] = 0.0f;
        vehicle.nodes.posX[1] = 1.2f;

        BeamContainer beams = vehicle.normalBeams;
        beams.count = 1;
        beams.node1[0] = 0;
        beams.node2[0] = 1;
        beams.restLength[0] = 1.0f;
        beams.actuationRatio[0] = 1.0f;
        beams.spring[0] = 200.0f;
        beams.damp[0] = 0.0f;
        beams.deform[0] = 1.0f;
        beams.maxDeform[0] = 100.0f;
        beams.strength[0] = 1_000_000.0f;
        beams.broken[0] = false;
        beams.breakGroupType[0] = 0;
        beams.disableTriangleBreaking[0] = false;
        beams.wheelId[0] = -1;

        // First solve: 40 N of spring load against a 1 N yield surface.
        vehicle.solveInternalForces(1.0e-3f, 1.0f);

        assertEquals(1.0975f, beams.restLength[0], 1.0e-3f,
                "over-yield load must permanently lengthen the beam's rest length");
        assertEquals(20.5f, beams.deform[0], 1.0e-3f,
                "the yield surface must harden towards the load");
        assertFalse(beams.broken[0], "an ordinary over-yield load must not break the beam");

        // Second solve at unchanged geometry: the hardened yield surface now
        // absorbs the elastic load, so state must be stable.
        vehicle.solveInternalForces(1.0e-3f, 1.0f);
        assertEquals(1.0975f, beams.restLength[0], 1.0e-3f);
        assertEquals(20.5f, beams.deform[0], 1.0e-3f);
        assertFalse(beams.broken[0]);

        // Stretch the beam further: it must yield again and harden once more.
        vehicle.nodes.posX[1] = 1.5f;
        vehicle.solveInternalForces(1.0e-3f, 1.0f);
        assertTrue(beams.restLength[0] > 1.0975f,
                "a larger stretch must further lengthen the rest state");
        assertTrue(beams.deform[0] > 20.5f,
                "a larger stretch must further harden the yield surface");
        assertFalse(beams.broken[0]);
    }
}
