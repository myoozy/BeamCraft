package me.mzy.beamcraft.client.physics.powertrain;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

class DctCouplingSolverTest {
    @Test
    void overlappingClutchesShareTheSameTwoEndpointInertias() {
        DctGearboxContainer state = new DctGearboxContainer();
        state.allocate(1);

        DctCouplingSolver.solveInto(0.0005f, 100.0f, 0.0f,
                0.2f, 0.1f, 1.0f, 0.65f,
                1600.0f, 0.15f, 0.15f, 400.0f,
                0.5f, 0.5f, state, 0);

        assertTrue(Float.isFinite(state.clutchTorque1[0]));
        assertTrue(Float.isFinite(state.clutchTorque2[0]));
        assertTrue(state.clutchTorque1[0] > 0.0f);
        assertTrue(state.clutchTorque2[0] > 0.0f);
        assertTrue(Math.abs(state.clutchTorque1[0]) <= 200.0f);
        assertTrue(Math.abs(state.clutchTorque2[0]) <= 200.0f);
    }
}
