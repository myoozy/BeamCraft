package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Per-unit friction-clutch SoA. One row per compiled engine→clutch unit, kept in the
 * same order as {@link CombustionEngineContainer} so the solver indexes both by unit id.
 *
 * <p>{@code clutchTorque} is written by {@link ImplicitClutchSolver#solveInto} and read
 * back by the solver to load the engine and the driven wheel paths.
 */
public final class FrictionClutchContainer {
    public float[] clutchCapacity = new float[0];
    public float[] clutchSpring = new float[0];
    public float[] clutchDampingRatio = new float[0];
    public float[] clutchAngle = new float[0];
    public float[] clutchTorque = new float[0];

    public void allocate(int units) {
        clutchCapacity = new float[units];
        clutchSpring = new float[units];
        clutchDampingRatio = new float[units];
        clutchAngle = new float[units];
        clutchTorque = new float[units];
    }

    /** Resets every array so the container holds zero units. */
    public void clear() {
        allocate(0);
    }
}
