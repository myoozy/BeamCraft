package me.mzy.beamcraft.client.physics.powertrain;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ImplicitCouplingSolverTest {
    @Test
    void implicitSolveMatchesBackwardEulerEquation() {
        float dt = 0.01f;
        float slip = 20.0f;
        float ja = 0.2f;
        float jb = 1.8f;
        float stiffness = 120.0f;
        float dampingRatio = 0.35f;

        ImplicitCouplingSolver.Result result = ImplicitCouplingSolver.solve(
                dt, slip, ja, jb, stiffness, dampingRatio, 10_000.0f, 1.0f, 0.1f);

        float reduced = ja * jb / (ja + jb);
        float damping = 2.0f * dampingRatio * (float) Math.sqrt(stiffness * reduced);
        float coefficient = stiffness * dt + damping;
        float expected = (stiffness * 0.1f + coefficient * slip)
                / (1.0f + coefficient * dt / reduced);
        assertEquals(expected, result.torque(), 1e-4f);
        assertEquals(0.3f, result.angleDifference(), 1e-6f);
        assertFalse(result.saturated());
    }

    @Test
    void capacityClampPreventsSpringWindup() {
        ImplicitCouplingSolver.Result result = ImplicitCouplingSolver.solve(
                0.01f, 1000.0f, 0.2f, 1.0f, 500.0f, 0.5f, 50.0f, 1.0f, 0.25f);
        assertEquals(50.0f, result.torque(), 1e-6f);
        assertEquals(0.25f, result.angleDifference(), 1e-6f);
        assertTrue(result.saturated());
    }

    @Test
    void disengagedCouplingClearsState() {
        ImplicitCouplingSolver.Result result = ImplicitCouplingSolver.solve(
                0.01f, 20.0f, 0.2f, 1.0f, 500.0f, 0.5f, 500.0f, 0.0f, 2.0f);
        assertEquals(0.0f, result.torque());
        assertEquals(0.0f, result.angleDifference());
    }
}
