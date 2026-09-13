package me.mzy.beamcraft.client.physics.powertrain;

/** Stateless local backward-Euler solve for a symmetric elastic friction coupling. */
public final class ImplicitCouplingSolver {
    private static final float MIN_INERTIA = 1e-7f;

    private ImplicitCouplingSolver() {
    }

    public static Result solve(float dt, float slip, float inertiaA, float inertiaB,
                               float stiffness, float dampingRatio, float capacity,
                               float engagement, float angleDifference) {
        float[] torque = {0.0f};
        float[] angle = {angleDifference};
        boolean saturated = solveInto(dt, slip, inertiaA, inertiaB, stiffness, dampingRatio,
                capacity, engagement, torque, angle, 0);
        return new Result(torque[0], angle[0], saturated);
    }

    /** Allocation-free hot-loop variant writing directly into a caller-owned runtime SoA. */
    public static boolean solveInto(float dt, float slip, float inertiaA, float inertiaB,
                                    float stiffness, float dampingRatio, float capacity, float engagement,
                                    float[] torqueState, float[] angleState, int index) {
        float lock = Math.clamp(engagement, 0.0f, 1.0f);
        if (dt <= 0.0f || lock <= 0.0f || capacity <= 0.0f) {
            torqueState[index] = 0.0f;
            angleState[index] = 0.0f;
            return false;
        }

        float ja = Math.max(inertiaA, MIN_INERTIA);
        float jb = Math.max(inertiaB, MIN_INERTIA);
        float reducedInertia = ja * jb / (ja + jb);
        float spring = Math.max(0.0f, stiffness);
        float damping = 2.0f * Math.max(0.0f, dampingRatio)
                * (float) Math.sqrt(spring * reducedInertia);
        float scaledSlip = slip * lock;
        float implicitCoefficient = spring * dt + damping;
        float denominator = 1.0f + implicitCoefficient * dt / reducedInertia;
        float unconstrainedTorque = (spring * angleState[index] + implicitCoefficient * scaledSlip) / denominator;
        float currentCapacity = capacity * lock;
        float torque = Math.clamp(unconstrainedTorque, -currentCapacity, currentCapacity);
        boolean saturated = Math.abs(unconstrainedTorque) > currentCapacity;

        // Do not wind up the virtual spring while friction capacity is saturated,
        // and erase stored angle as the coupling disengages.
        float nextAngle = angleState[index];
        if (!saturated) nextAngle += scaledSlip * dt;
        nextAngle *= lock;
        torqueState[index] = torque;
        angleState[index] = nextAngle;
        return saturated;
    }

    public record Result(float torque, float angleDifference, boolean saturated) {
    }
}
