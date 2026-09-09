package me.mzy.beamcraft.client.physics.powertrain;

/** Coupling solve between the primary and secondary reflected split-shaft inertias. */
public final class SplitShaftSolver {
    private static final float MIN_INERTIA = 1.0e-7f;

    private SplitShaftSolver() {
    }

    public static float solve(float dt, float primaryAV, float secondaryAV,
                              float primaryInertia, float secondaryInertia,
                              SplitShaftContainer state, int unit) {
        if (dt <= 0.0f || state.activeMode[unit] == SplitShaftContainer.MODE_DISCONNECTED) {
            clear(state, unit);
            return 0.0f;
        }
        if (state.activeMode[unit] == SplitShaftContainer.MODE_VISCOUS) {
            state.lockTorque[unit] = 0.0f;
            state.shaftAngle[unit] = 0.0f;
            float slip = primaryAV - secondaryAV;
            float magnitude = state.viscousCoef[unit]
                    * (float) Math.pow(Math.abs(slip), state.viscousExponent[unit]);
            float target = Math.clamp(Math.copySign(magnitude, slip),
                    -state.viscousCapacity[unit], state.viscousCapacity[unit]);
            float alpha = Math.clamp(dt * state.viscousSmoothing[unit], 0.0f, 1.0f);
            float torque = Math.fma(target - state.viscousTorque[unit], alpha,
                    state.viscousTorque[unit]);
            torque = clampToNoSlipImpulse(dt, slip, primaryInertia, secondaryInertia, torque);
            state.viscousTorque[unit] = torque;
            return torque;
        }

        state.viscousTorque[unit] = 0.0f;
        ImplicitClutchSolver.solveInto(
                dt, primaryAV - secondaryAV, primaryInertia, secondaryInertia,
                state.lockSpring[unit], state.lockDampingRatio[unit], state.lockCapacity[unit],
                state.clutchRatio[unit], state.lockTorque, state.shaftAngle, unit);
        return state.lockTorque[unit];
    }

    private static float clampToNoSlipImpulse(float dt, float slip, float primaryInertia,
                                               float secondaryInertia, float torque) {
        float jp = Math.max(primaryInertia, MIN_INERTIA);
        float js = Math.max(secondaryInertia, MIN_INERTIA);
        float reduced = jp * js / (jp + js);
        float limit = Math.abs(slip) * reduced / dt;
        return Math.clamp(torque, -limit, limit);
    }

    public static void clear(SplitShaftContainer state, int unit) {
        state.lockTorque[unit] = 0.0f;
        state.shaftAngle[unit] = 0.0f;
        state.viscousTorque[unit] = 0.0f;
    }
}
