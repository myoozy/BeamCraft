package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Engine crank-reaction and torsion-reactor node SoA. Crank reactions are flat
 * {@code reactionNodes} ranges per unit ({@code reactionStart}/{@code reactionCount});
 * each torsionReactor further adds a gain and its own node range
 * ({@code reactorNodeStart}/{@code reactorNodeCount}) into the same node pool.
 *
 * <p>All node references are resolved to {@code NodeContainer} indices at build time so
 * the substep only copies them into {@link TorqueReactionSolver}.
 */
public final class TorqueReactionContainer {
    public int[] reactionStart = new int[0];
    public byte[] reactionCount = new byte[0];
    public int[] reactionNodes = new int[0];
    public int[] reactorStart = new int[0];
    public short[] reactorCount = new short[0];
    public float[] reactorGain = new float[0];
    public int[] reactorNodeStart = new int[0];
    public byte[] reactorNodeCount = new byte[0];

    public void allocate(int units, int reactionNodeCount, int reactors) {
        reactionStart = new int[units];
        reactionCount = new byte[units];
        reactionNodes = new int[reactionNodeCount];
        reactorStart = new int[units];
        reactorCount = new short[units];
        reactorGain = new float[reactors];
        reactorNodeStart = new int[reactors];
        reactorNodeCount = new byte[reactors];
    }

    /** Resets every array so the container holds zero reactions. */
    public void clear() {
        allocate(0, 0, 0);
    }
}
