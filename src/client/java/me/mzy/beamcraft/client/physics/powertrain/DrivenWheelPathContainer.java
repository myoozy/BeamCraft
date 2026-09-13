/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
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
    public static final byte FLAG_GEARBOX = 1;
    public static final byte FLAG_RANGE_BOX = 2;
    public int[] pathStart = new int[0];
    public short[] pathCount = new short[0];
    public int[] pathWheel = new int[0];
    public float[] pathGain = new float[0];
    public byte[] pathFlags = new byte[0];

    public void allocate(int units, int paths) {
        pathStart = new int[units];
        pathCount = new short[units];
        pathWheel = new int[paths];
        pathGain = new float[paths];
        pathFlags = new byte[paths];
    }

    /** Resets every array so the container holds zero paths. */
    public void clear() {
        allocate(0, 0);
    }
}
