package me.mzy.beamcraft.client.physics.powertrain;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TorqueConverterSolverTest {
    @Test
    void stalledTurbineReceivesPositiveTorqueMultiplication() {
        TorqueConverterContainer state = state();

        TorqueConverterSolver.solveInto(
                0.0005f, 200.0f, 0.0f, 0.25f, 1.0f,
                0.9f, 1.8f, 10.0f, 0.31f, 1000.0f, 0.0f,
                state, 0);

        assertTrue(state.inputTorque[0] > 0.0f);
        assertEquals(1.8f * state.inputTorque[0], state.outputTorque[0], 1e-4f);
        assertEquals(0.0f, state.lockupTorque[0], 1e-6f);
        assertEquals(0.0f, state.speedRatio[0], 1e-6f);
    }

    @Test
    void externalLockupRatioEngagesImplicitClutch() {
        TorqueConverterContainer state = state();

        TorqueConverterSolver.solveInto(
                0.0005f, 200.0f, 100.0f, 0.25f, 1.0f,
                0.9f, 1.8f, 0.0f, 0.31f, 1000.0f, 1.0f,
                state, 0);

        assertTrue(state.lockupTorque[0] > 0.0f);
        assertEquals(state.lockupTorque[0], state.inputTorque[0], 1e-6f);
        assertEquals(state.lockupTorque[0], state.outputTorque[0], 1e-6f);
    }

    @Test
    void zeroLockupCommandClearsStoredLockupState() {
        TorqueConverterContainer state = state();
        state.lockupAngle[0] = 0.2f;

        TorqueConverterSolver.solveInto(
                0.0005f, 200.0f, 100.0f, 0.25f, 1.0f,
                0.9f, 1.8f, 0.0f, 0.31f, 1000.0f, 0.0f,
                state, 0);

        assertEquals(0.0f, state.lockupTorque[0], 1e-6f);
        assertEquals(0.0f, state.lockupAngle[0], 1e-6f);
    }

    @Test
    void combinedFluidAndLockupImpulseDoesNotReverseSlip() {
        TorqueConverterContainer state = state();
        float dt = 0.01f;
        float inputAV = 200.0f;
        float outputAV = 0.0f;
        float inputInertia = 0.1f;
        float outputInertia = 0.1f;

        TorqueConverterSolver.solveInto(
                dt, inputAV, outputAV, inputInertia, outputInertia,
                0.9f, 1.8f, 100.0f, 0.5f, 100000.0f, 1.0f,
                state, 0);

        float nextInputAV = inputAV - dt * state.inputTorque[0] / inputInertia;
        float nextOutputAV = outputAV + dt * state.outputTorque[0] / outputInertia;
        assertTrue(nextInputAV - nextOutputAV >= -1e-4f,
                "coupling impulse reversed the pump/turbine slip");
    }

    private static TorqueConverterContainer state() {
        TorqueConverterContainer state = new TorqueConverterContainer();
        state.allocate(1);
        state.lockupCapacity[0] = 500.0f;
        state.lockupSpring[0] = 4000.0f;
        state.lockupDampingRatio[0] = 0.15f;
        return state;
    }
}
