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
    void lockedModeAdvancesShaftAngleFromThePostImpulseSlip() {
        SplitShaftContainer state = state();
        state.activeMode[0] = SplitShaftContainer.MODE_LOCKED;
        state.lockSpring[0] = 10.0f;
        state.lockDampingRatio[0] = 0.0f;
        state.lockCapacity[0] = 10_000.0f;

        float torque = SplitShaftSolver.solve(0.1f, 10.0f, 0.0f,
                1.0f, 1.0f, state, 0);
        float postSlip = 10.0f - 0.1f * torque / 0.5f;

        assertEquals(0.1f * postSlip, state.shaftAngle[0], 1.0e-6f,
                "the stored spring state must use the same locally solved slip as torque");
        assertTrue(state.shaftAngle[0] < 1.0f,
                "explicit old-slip integration would have stored an angle of exactly one radian");
    }

    @Test
    void lockedModeReleasesStoredAngleThroughAxleMotionWithoutInputTorque() {
        SplitShaftContainer state = state();
        state.activeMode[0] = SplitShaftContainer.MODE_LOCKED;
        state.lockSpring[0] = 100.0f;
        state.lockDampingRatio[0] = 0.2f;
        state.lockCapacity[0] = 10_000.0f;
        state.shaftAngle[0] = 0.1f;

        float initialAngle = state.shaftAngle[0];
        float slip = 0.0f;
        for (int i = 0; i < 500; i++) {
            float torque = SplitShaftSolver.solve(0.001f, slip, 0.0f,
                    1.0f, 1.0f, state, 0);
            // Integrate the equal-and-opposite axle response represented by the reduced
            // inertia before the following coupling step.
            slip -= 0.001f * torque / 0.5f;
        }

        assertTrue(Math.abs(state.shaftAngle[0]) < initialAngle * 0.5f,
                "stored transfer-case wind-up must decay when the axles are free to respond");
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
