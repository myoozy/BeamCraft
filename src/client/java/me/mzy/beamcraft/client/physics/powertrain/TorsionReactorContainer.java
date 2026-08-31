package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Compiled torsion-reactor parameters, one row per {@code torsionReactor} device that
 * carries a connected wheel or reaction nodes. Build-time only: the gear ratio feeds
 * {@link PowertrainTopologyContainer#deviceRatio} and the resolved reaction-node ranges are
 * folded into {@link TorqueReactionContainer} by {@link PowertrainCompiler}.
 */
public final class TorsionReactorContainer {
    public int count;
    public int[] device = new int[0];
    public float[] gearRatio = new float[0];
    public String[] connectedWheel = new String[0];
    public float[] friction = new float[0];
    public float[] dynamicFriction = new float[0];
    public float[] torqueLossCoef = new float[0];

    public void clear() {
        count = 0;
        device = new int[0];
        gearRatio = new float[0];
        connectedWheel = new String[0];
        friction = new float[0];
        dynamicFriction = new float[0];
        torqueLossCoef = new float[0];
    }
}
