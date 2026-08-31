package me.mzy.beamcraft.client.physics.powertrain;

/** Stateless local backward-Euler solve for one elastic clutch boundary. */
public final class ImplicitClutchSolver {
    private static final float MIN_INERTIA = 1e-7f;

    private ImplicitClutchSolver() {
    }

    public static Result solve(float dt, float slip, float engineInertia, float drivelineInertia,
                               float stiffness, float dampingRatio, float capacity,
                               float engagement, float angleDifference) {
        float[] torque = {0.0f};
        float[] angle = {angleDifference};
        boolean saturated = solveInto(dt, slip, engineInertia, drivelineInertia, stiffness, dampingRatio,
                capacity, engagement, torque, angle, 0);
        return new Result(torque[0], angle[0], saturated);
    }

    /** Allocation-free hot-loop variant writing directly into the runtime SoA. */
    public static boolean solveInto(float dt, float slip, float engineInertia, float drivelineInertia,
                                    float stiffness, float dampingRatio, float capacity, float engagement,
                                    float[] torqueState, float[] angleState, int index) {
        float lock = clamp(engagement, 0.0f, 1.0f);
        if (dt <= 0.0f || lock <= 0.0f || capacity <= 0.0f) {
            torqueState[index] = 0.0f;
            angleState[index] = 0.0f;
            return false;
        }

        float je = Math.max(engineInertia, MIN_INERTIA);
        float jd = Math.max(drivelineInertia, MIN_INERTIA);
        float reducedInertia = je * jd / (je + jd);
        float spring = Math.max(0.0f, stiffness);
        float damping = 2.0f * Math.max(0.0f, dampingRatio)
                * (float) Math.sqrt(spring * reducedInertia);
        float scaledSlip = slip * lock;
        float implicitCoefficient = spring * dt + damping;
        float denominator = 1.0f + implicitCoefficient * dt / reducedInertia;
        float unconstrainedTorque = (spring * angleState[index] + implicitCoefficient * scaledSlip) / denominator;
        float currentCapacity = capacity * lock;
        float torque = clamp(unconstrainedTorque, -currentCapacity, currentCapacity);
        boolean saturated = Math.abs(unconstrainedTorque) > currentCapacity;

        // Match the KinetiForge spring model: do not wind up the virtual spring
        // while the friction capacity is saturated, and erase stored angle as
        // the pedal disengages.
        float nextAngle = angleState[index];
        if (!saturated) nextAngle += scaledSlip * dt;
        nextAngle *= lock;
        torqueState[index] = torque;
        angleState[index] = nextAngle;
        return saturated;
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    public record Result(float torque, float angleDifference, boolean saturated) {
    }
}
