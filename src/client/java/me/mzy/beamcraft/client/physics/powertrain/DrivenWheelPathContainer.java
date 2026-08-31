package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Driven-wheel paths SoA. One {@code (wheel, gain)} pair per rigid branch from a
 * compiled unit's clutch output down to a connected wheel; {@code pathStart}/{@code pathCount}
 * index the flat {@code pathWheel}/{@code pathGain} arrays per unit.
 *
 * <p>This is the only runtime data the solver reads to reflect wheel spin back into the
 * engine and to push clutch torque out to the wheels.
 */
public final class DrivenWheelPathContainer {
    public int[] pathStart = new int[0];
    public short[] pathCount = new short[0];
    public int[] pathWheel = new int[0];
    public float[] pathGain = new float[0];

    /** Resets every array so the container holds zero paths. */
    public void clear() {
        pathStart = new int[0];
        pathCount = new short[0];
        pathWheel = new int[0];
        pathGain = new float[0];
    }
}
