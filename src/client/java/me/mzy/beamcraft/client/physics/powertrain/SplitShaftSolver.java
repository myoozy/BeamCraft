package me.mzy.beamcraft.client.physics.powertrain;

/** Coupling solve between the primary and secondary reflected split-shaft inertias. */
public final class SplitShaftSolver {
    private static final float MIN_INERTIA = 1.0e-7f;

    private SplitShaftSolver() {
    }

    public static float solve(float dt, float primaryAV, float secondaryAV,
                              float primaryInertia, float secondaryInertia,
                              SplitShaftContainer state, int split) {
        if (dt <= 0.0f || state.activeMode[split] == SplitShaftContainer.MODE_DISCONNECTED) {
            clear(state, split);
            return 0.0f;
        }
        if (state.activeMode[split] == SplitShaftContainer.MODE_VISCOUS) {
            state.lockTorque[split] = 0.0f;
            state.shaftAngle[split] = 0.0f;
            float slip = primaryAV - secondaryAV;
            float magnitude = state.viscousCoef[split]
                    * (float) Math.pow(Math.abs(slip), state.viscousExponent[split]);
            float target = Math.clamp(Math.copySign(magnitude, slip),
                    -state.viscousCapacity[split], state.viscousCapacity[split]);
            float alpha = Math.clamp(dt * state.viscousSmoothing[split], 0.0f, 1.0f);
            float torque = Math.fma(target - state.viscousTorque[split], alpha,
                    state.viscousTorque[split]);
            torque = clampToNoSlipImpulse(dt, slip, primaryInertia, secondaryInertia, torque);
            state.viscousTorque[split] = torque;
            return torque;
        }

        state.viscousTorque[split] = 0.0f;
        ImplicitCouplingSolver.solveInto(
                dt, primaryAV - secondaryAV, primaryInertia, secondaryInertia,
                state.lockSpring[split], state.lockDampingRatio[split], state.lockCapacity[split],
                state.clutchRatio[split], state.lockTorque, state.shaftAngle, split);
        return state.lockTorque[split];
    }

    private static float clampToNoSlipImpulse(float dt, float slip, float primaryInertia,
                                               float secondaryInertia, float torque) {
        float jp = Math.max(primaryInertia, MIN_INERTIA);
        float js = Math.max(secondaryInertia, MIN_INERTIA);
        float reduced = jp * js / (jp + js);
        float limit = Math.abs(slip) * reduced / dt;
        return Math.clamp(torque, -limit, limit);
    }

    public static void clear(SplitShaftContainer state, int split) {
        state.lockTorque[split] = 0.0f;
        state.shaftAngle[split] = 0.0f;
        state.viscousTorque[split] = 0.0f;
    }
}
