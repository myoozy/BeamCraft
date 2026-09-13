package me.mzy.beamcraft.client.physics.powertrain;

/** Allocation-free coupled backward-Euler solve for the two DCT clutch paths. */
public final class DctCouplingSolver {
    private static final float MIN_INERTIA = 1.0e-7f;

    private DctCouplingSolver() {
    }

    public static void solveInto(float dt, float engineAV, float baseDrivelineAV,
                                 float engineInertia, float baseCompliance,
                                 float ratioFactor1, float ratioFactor2,
                                 float stiffness, float dampingRatio1, float dampingRatio2,
                                 float capacity, float engagement1, float engagement2,
                                 DctGearboxContainer state, int unit) {
        float e1 = Math.clamp(engagement1, 0.0f, 1.0f);
        float e2 = Math.clamp(engagement2, 0.0f, 1.0f);
        if (dt <= 0.0f || capacity <= 0.0f || (e1 <= 0.0f && e2 <= 0.0f)) {
            state.clutchTorque1[unit] = 0.0f;
            state.clutchTorque2[unit] = 0.0f;
            if (capacity <= 0.0f || e1 <= 0.0f) state.clutchAngle1[unit] = 0.0f;
            if (capacity <= 0.0f || e2 <= 0.0f) state.clutchAngle2[unit] = 0.0f;
            return;
        }

        float invEngineInertia = 1.0f / Math.max(engineInertia, MIN_INERTIA);
        float compliance = Math.max(0.0f, baseCompliance);
        float a11 = invEngineInertia + ratioFactor1 * ratioFactor1 * compliance;
        float a12 = invEngineInertia + ratioFactor1 * ratioFactor2 * compliance;
        float a22 = invEngineInertia + ratioFactor2 * ratioFactor2 * compliance;
        float spring = Math.max(0.0f, stiffness);

        float reduced1 = 1.0f / Math.max(a11, MIN_INERTIA);
        float reduced2 = 1.0f / Math.max(a22, MIN_INERTIA);
        float damp1 = 2.0f * Math.max(0.0f, dampingRatio1)
                * (float) Math.sqrt(spring * reduced1);
        float damp2 = 2.0f * Math.max(0.0f, dampingRatio2)
                * (float) Math.sqrt(spring * reduced2);
        float h1 = spring * dt + damp1;
        float h2 = spring * dt + damp2;
        float slip1 = engineAV - ratioFactor1 * baseDrivelineAV;
        float slip2 = engineAV - ratioFactor2 * baseDrivelineAV;

        float m11 = 1.0f + h1 * e1 * dt * a11;
        float m12 = h1 * e1 * dt * a12;
        float m21 = h2 * e2 * dt * a12;
        float m22 = 1.0f + h2 * e2 * dt * a22;
        float b1 = spring * state.clutchAngle1[unit] + h1 * e1 * slip1;
        float b2 = spring * state.clutchAngle2[unit] + h2 * e2 * slip2;

        float determinant = m11 * m22 - m12 * m21;
        float torque1;
        float torque2;
        if (Math.abs(determinant) > 1.0e-9f) {
            torque1 = (b1 * m22 - m12 * b2) / determinant;
            torque2 = (m11 * b2 - b1 * m21) / determinant;
        } else {
            torque1 = b1 / Math.max(m11, 1.0e-7f);
            torque2 = b2 / Math.max(m22, 1.0e-7f);
        }

        float limit1 = Math.max(0.0f, capacity) * e1;
        float limit2 = Math.max(0.0f, capacity) * e2;
        boolean saturated1 = Math.abs(torque1) > limit1;
        boolean saturated2 = Math.abs(torque2) > limit2;
        if (saturated1 || saturated2) {
            float excess1 = limit1 > 0.0f ? Math.abs(torque1) / limit1 : Float.POSITIVE_INFINITY;
            float excess2 = limit2 > 0.0f ? Math.abs(torque2) / limit2 : Float.POSITIVE_INFINITY;
            if (saturated1 && (!saturated2 || excess1 >= excess2)) {
                torque1 = Math.clamp(torque1, -limit1, limit1);
                torque2 = Math.clamp((b2 - m21 * torque1) / Math.max(m22, 1.0e-7f), -limit2, limit2);
            } else {
                torque2 = Math.clamp(torque2, -limit2, limit2);
                torque1 = Math.clamp((b1 - m12 * torque2) / Math.max(m11, 1.0e-7f), -limit1, limit1);
            }
        }

        float nextSlip1 = slip1 - dt * (a11 * torque1 + a12 * torque2);
        float nextSlip2 = slip2 - dt * (a12 * torque1 + a22 * torque2);
        state.clutchTorque1[unit] = torque1;
        state.clutchTorque2[unit] = torque2;
        state.clutchAngle1[unit] = e1 > 0.0f && Math.abs(torque1) < limit1
                ? (state.clutchAngle1[unit] + e1 * nextSlip1 * dt) * e1 : 0.0f;
        state.clutchAngle2[unit] = e2 > 0.0f && Math.abs(torque2) < limit2
                ? (state.clutchAngle2[unit] + e2 * nextSlip2 * dt) * e2 : 0.0f;
    }

}
