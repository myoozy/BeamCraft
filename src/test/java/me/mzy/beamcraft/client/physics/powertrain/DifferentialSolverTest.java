/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 */
package me.mzy.beamcraft.client.physics.powertrain;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DifferentialSolverTest {
    @Test
    void viscousCouplingIsCapacityLimitedAndCannotReverseSlip() {
        DifferentialContainer state = state();
        state.activeMode[0] = DifferentialContainer.MODE_VISCOUS;
        state.viscousCoef[0] = 1000.0f;
        state.viscousCapacity[0] = 5000.0f;
        state.viscousSmoothing[0] = 10000.0f;

        float torque = DifferentialSolver.solve(0.1f, 10.0f, 0.0f,
                1.0f, 1.0f, 0.0f, state, 0);

        assertEquals(50.0f, torque, 1.0e-4f,
                "the reduced-inertia impulse may eliminate, but never reverse, this step's slip");
    }

    @Test
    void lsdUsesDifferentDriveAndCoastTorqueSensing() {
        DifferentialContainer state = state();
        state.activeMode[0] = DifferentialContainer.MODE_LSD;
        state.lsdPreload[0] = 0.0f;
        state.lsdLockCoef[0] = 0.8f;
        state.lsdRevLockCoef[0] = 0.1f;
        state.lockSpring[0] = 10_000.0f;

        float driveTorque = runUntilEstablished(state, 500.0f);
        DifferentialSolver.clear(state, 0);
        float coastTorque = runUntilEstablished(state, -500.0f);

        assertTrue(driveTorque > coastTorque * 2.0f,
                "drive locking should be stronger than coast locking for these JBeam coefficients");
        assertTrue(driveTorque <= 400.0f);
        assertTrue(coastTorque <= 50.0f);
    }

    @Test
    void activeLockHasAControllerFacingRatioAndNoImplicitController() {
        DifferentialContainer state = state();
        state.activeMode[0] = DifferentialContainer.MODE_ACTIVE_LOCK;
        state.activeLockCapacity[0] = 300.0f;

        float openCommand = DifferentialSolver.solve(0.001f, 20.0f, 0.0f,
                1.0f, 1.0f, 100.0f, state, 0);
        state.activeLockCoef[0] = 1.0f;
        float lockedCommand = DifferentialSolver.solve(0.001f, 20.0f, 0.0f,
                1.0f, 1.0f, 100.0f, state, 0);

        assertEquals(0.0f, openCommand);
        assertTrue(lockedCommand > 0.0f);
        assertTrue(lockedCommand <= 300.0f);
    }

    @Test
    void modeAliasesMatchBeamngDeviceModes() {
        assertEquals(DifferentialContainer.MODE_OPEN, DifferentialSolver.mode("torqueVectoring"));
        assertEquals(DifferentialContainer.MODE_LOCKED, DifferentialSolver.mode("dually"));
        assertEquals(DifferentialContainer.MODE_ACTIVE_LOCK, DifferentialSolver.mode("activeLock"));
    }

    private static float runUntilEstablished(DifferentialContainer state, float inputTorque) {
        float torque = 0.0f;
        for (int i = 0; i < 100; i++) {
            torque = DifferentialSolver.solve(0.001f, 20.0f, 0.0f,
                    100.0f, 100.0f, inputTorque, state, 0);
        }
        return torque;
    }

    private static DifferentialContainer state() {
        DifferentialContainer state = new DifferentialContainer();
        state.allocate(1, 1, 0, 0);
        state.lockCapacity[0] = 1000.0f;
        state.lockSpring[0] = 8000.0f;
        state.lockDampingRatio[0] = 0.1f;
        state.viscousCoef[0] = 5.0f;
        state.viscousCapacity[0] = 50.0f;
        state.viscousExponent[0] = 1.0f;
        state.viscousSmoothing[0] = 25.0f;
        return state;
    }
}
