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

    /** Resets every array so the container holds zero units. */
    public void clear() {
        clutchCapacity = new float[0];
        clutchSpring = new float[0];
        clutchDampingRatio = new float[0];
        clutchAngle = new float[0];
        clutchTorque = new float[0];
    }
}
