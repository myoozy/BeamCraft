package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Allocation-free local solve for a hydrodynamic torque converter with an optional
 * lock-up clutch. The fluid coupling follows the usual converter similarity law
 * ({@code torque proportional to density * diameter^5 * angularVelocity^2}); its
 * dimensionless capacity smoothly approaches zero as pump and turbine speeds converge.
 */
public final class TorqueConverterSolver {
    private static final float FLUID_DENSITY = 844.0f;
    private static final float CAPACITY_SCALE = 0.004f;
    private static final float MIN_AV = 1.0e-4f;
    private static final float MIN_INERTIA = 1.0e-7f;

    private TorqueConverterSolver() {
    }

    public static void solveInto(float dt, float inputAV, float outputAV,
                                 float inputInertia, float outputInertia,
                                 float couplingAVRatio, float stallTorqueRatio,
                                 float converterStiffness, float converterDiameter,
                                 float converterTorqueLimit, float lockupRatio,
                                 TorqueConverterContainer state, int index) {
        if (dt <= 0.0f) {
            state.inputTorque[index] = 0.0f;
            state.outputTorque[index] = 0.0f;
            state.lockupTorque[index] = 0.0f;
            return;
        }

        float safeCouplingRatio = Math.max(0.05f, couplingAVRatio);
        float speedRatio = Math.abs(inputAV) > MIN_AV ? outputAV / inputAV : 1.0f;
        state.speedRatio[index] = speedRatio;

        float ratioError = (1.0f - speedRatio) / safeCouplingRatio;
        float shape = (float) Math.tanh(Math.max(0.0f, converterStiffness) * ratioError);
        float diameter = Math.max(0.0f, converterDiameter);
        float diameter2 = diameter * diameter;
        float diameter5 = diameter2 * diameter2 * diameter;
        float pumpTorque = CAPACITY_SCALE * FLUID_DENSITY * diameter5
                * inputAV * Math.abs(inputAV) * shape;
        float torqueLimit = Math.max(0.0f, converterTorqueLimit);
        if (torqueLimit > 0.0f) pumpTorque = Math.clamp(pumpTorque, -torqueLimit, torqueLimit);

        float stallRatio = Math.max(1.0f, stallTorqueRatio);
        float multiplication = Math.clamp(
                stallRatio - (stallRatio - 1.0f) * speedRatio / safeCouplingRatio,
                1.0f, stallRatio);

        // Bound a fluid impulse that opposes slip so one substep cannot reverse that slip.
        // The stator allows turbine torque to differ from impeller reaction torque, hence
        // the multiplication term in the output-side inverse inertia.
        float slip = inputAV - outputAV;
        if (pumpTorque * slip > 0.0f) {
            float inverseEffectiveInertia = 1.0f / Math.max(inputInertia, MIN_INERTIA)
                    + multiplication / Math.max(outputInertia, MIN_INERTIA);
            float noOvershootTorque = Math.abs(slip) / Math.max(dt * inverseEffectiveInertia, 1.0e-9f);
            pumpTorque = Math.copySign(Math.min(Math.abs(pumpTorque), noOvershootTorque), pumpTorque);
        }

        float slipAfterFluid = slip - dt * pumpTorque * (
                1.0f / Math.max(inputInertia, MIN_INERTIA)
                        + multiplication / Math.max(outputInertia, MIN_INERTIA));
        ImplicitCouplingSolver.solveInto(
                dt, slipAfterFluid, inputInertia, outputInertia,
                state.lockupSpring[index], state.lockupDampingRatio[index], state.lockupCapacity[index],
                lockupRatio, state.lockupTorque, state.lockupAngle, index);
        float lockupTorque = state.lockupTorque[index];
        state.inputTorque[index] = pumpTorque + lockupTorque;
        state.outputTorque[index] = pumpTorque * multiplication + lockupTorque;
    }
}
