package me.mzy.beamcraft.client.physics.powertrain;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SplitShaftSolverTest {
    @Test
    void disconnectedModeTransfersNoTorqueAndClearsState() {
        SplitShaftContainer state = state();
        state.activeMode[0] = SplitShaftContainer.MODE_DISCONNECTED;
        state.lockTorque[0] = 12.0f;
        state.shaftAngle[0] = 0.2f;
        state.viscousTorque[0] = 8.0f;

        assertEquals(0.0f, SplitShaftSolver.solve(0.0005f, 100, 0, 1, 1, state, 0));
        assertEquals(0.0f, state.lockTorque[0]);
        assertEquals(0.0f, state.shaftAngle[0]);
        assertEquals(0.0f, state.viscousTorque[0]);
    }

    @Test
    void lockedModeProducesTorqueThatOpposesPrimarySecondarySlip() {
        SplitShaftContainer state = state();
        state.activeMode[0] = SplitShaftContainer.MODE_LOCKED;

        float torque = SplitShaftSolver.solve(0.0005f, 100, 20, 1, 2, state, 0);

        assertTrue(torque > 0.0f);
        assertTrue(torque <= state.lockCapacity[0]);
    }

    @Test
    void viscousModeIsSmoothedCappedAndCannotReverseSlipInOneStep() {
        SplitShaftContainer state = state();
        state.activeMode[0] = SplitShaftContainer.MODE_VISCOUS;
        state.viscousCoef[0] = 1000.0f;
        state.viscousCapacity[0] = 5000.0f;
        state.viscousSmoothing[0] = 10000.0f;

        float torque = SplitShaftSolver.solve(0.1f, 10, 0, 1, 1, state, 0);

        assertEquals(50.0f, torque, 1.0e-5f,
                "the no-slip impulse limit is reducedInertia * slip / dt");
    }

    private static SplitShaftContainer state() {
        SplitShaftContainer state = new SplitShaftContainer();
        state.allocate(0, 1, 0);
        state.clutchRatio[0] = 1.0f;
        state.lockCapacity[0] = 1000.0f;
        state.lockSpring[0] = 8000.0f;
        state.lockDampingRatio[0] = 0.15f;
        state.viscousCoef[0] = 10.0f;
        state.viscousCapacity[0] = 100.0f;
        state.viscousExponent[0] = 1.0f;
        state.viscousSmoothing[0] = 25.0f;
        return state;
    }
}
