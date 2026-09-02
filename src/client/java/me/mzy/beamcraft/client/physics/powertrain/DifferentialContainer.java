package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Compiled differential parameters, one row per {@code differential}/{@code openDifferential}
 * device. Build-time only: the gear ratio feeds {@link PowertrainTopologyContainer#deviceRatio} and
 * {@code diffTorqueSplit} is used by {@link PowertrainCompiler} to split wheel paths.
 */
public final class DifferentialContainer {
    public int count;
    public int[] device = new int[0];
    public float[] gearRatio = new float[0];
    public float[] diffTorqueSplit = new float[0];
    public String[] diffType = new String[0];
    public float[] friction = new float[0];
    public float[] dynamicFriction = new float[0];
    public float[] torqueLossCoef = new float[0];

    public void allocate(int size) {
        count = size;
        device = new int[size];
        gearRatio = new float[size];
        diffTorqueSplit = new float[size];
        diffType = new String[size];
        friction = new float[size];
        dynamicFriction = new float[size];
        torqueLossCoef = new float[size];
    }

    public void clear() {
        allocate(0);
    }
}
