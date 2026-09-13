/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Engine crank-reaction and torsion-reactor node SoA. Crank reactions are flat
 * {@code reactionNodes} ranges per unit ({@code reactionStart}/{@code reactionCount});
 * each torsionReactor further adds its own node range and a flat torque-expression
 * range.  Expression terms either reference the unit coupler ({@code termSplit == -1})
 * or the actual transferred torque of one compiled split shaft.
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
    public int[] reactorNodeStart = new int[0];
    public byte[] reactorNodeCount = new byte[0];
    public int[] reactorTermStart = new int[0];
    public short[] reactorTermCount = new short[0];
    public int[] termSplit = new int[0];
    public float[] termGain = new float[0];
    public byte[] termFlags = new byte[0];

    public void allocate(int units, int reactionNodeCount, int reactors, int terms) {
        reactionStart = new int[units];
        reactionCount = new byte[units];
        reactionNodes = new int[reactionNodeCount];
        reactorStart = new int[units];
        reactorCount = new short[units];
        reactorNodeStart = new int[reactors];
        reactorNodeCount = new byte[reactors];
        reactorTermStart = new int[reactors];
        reactorTermCount = new short[reactors];
        termSplit = new int[terms];
        termGain = new float[terms];
        termFlags = new byte[terms];
    }

    /** Resets every array so the container holds zero reactions. */
    public void clear() {
        allocate(0, 0, 0, 0);
    }
}
