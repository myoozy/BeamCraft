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
        return solveLocked(dt, primaryAV - secondaryAV, primaryInertia,
                secondaryInertia, state, split);
    }

    /**
     * Locally implicit linear locked-shaft solve.  Torque and the stored shaft angle use
     * the same post-impulse slip; the generic clutch solver historically calculated an
     * implicit torque but then advanced this angle from the old slip, which could retain
     * artificial transfer-case wind-up after the axle speeds had already converged.
     *
     * <p>The spring remains linear here.  BeamNG's squared-angle spring law can replace
     * the constitutive term independently without changing the two-inertia coupling.
     */
    private static float solveLocked(float dt, float slip, float primaryInertia,
                                     float secondaryInertia, SplitShaftContainer state,
                                     int split) {
        float lock = Math.clamp(state.clutchRatio[split], 0.0f, 1.0f);
        float capacity = Math.max(0.0f, state.lockCapacity[split]);
        if (lock <= 0.0f || capacity <= 0.0f) {
            state.lockTorque[split] = 0.0f;
            state.shaftAngle[split] = 0.0f;
            return 0.0f;
        }

        float jp = Math.max(primaryInertia, MIN_INERTIA);
        float js = Math.max(secondaryInertia, MIN_INERTIA);
        float reducedInertia = jp * js / (jp + js);
        float spring = Math.max(0.0f, state.lockSpring[split]);
        float damping = 2.0f * Math.max(0.0f, state.lockDampingRatio[split])
                * (float) Math.sqrt(spring * reducedInertia);

        // nextAngle = lock * (oldAngle + dt * lock * postSlip)
        // torque = spring * nextAngle + damping * lock * postSlip
        // postSlip = slip - dt * torque / reducedInertia
        float response = spring * dt * lock * lock + damping * lock;
        float numerator = spring * lock * state.shaftAngle[split] + response * slip;
        float denominator = 1.0f + response * dt / reducedInertia;
        float unconstrainedTorque = numerator / denominator;
        float torque = Math.clamp(unconstrainedTorque, -capacity * lock, capacity * lock);
        float postSlip = slip - dt * torque / reducedInertia;
        float nextAngle = lock * (state.shaftAngle[split] + dt * lock * postSlip);

        // Linear counterpart of BeamNG's maxShaftAngle bound.  It prevents stored spring
        // energy from exceeding the configured lock capacity while still allowing a
        // saturated coupling to unwind on subsequent steps.
        if (spring > 1.0e-9f) {
            float maxAngle = capacity * lock / spring;
            nextAngle = Math.clamp(nextAngle, -maxAngle, maxAngle);
        } else {
            nextAngle = 0.0f;
        }
        state.lockTorque[split] = torque;
        state.shaftAngle[split] = nextAngle;
        return torque;
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
