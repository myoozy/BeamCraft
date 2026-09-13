/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Per-engine runtime state for the optional high/low range box downstream of the
 * primary gearbox. BeamNG exposes two modes: high is the smallest positive ratio
 * and low is the largest positive ratio.
 */
public final class RangeBoxContainer {
    public int unitCount;
    /** Topology device index, or -1 when this engine unit has no range box. */
    public int[] device = new int[0];
    public String[] deviceName = new String[0];
    /** Ratio baked into the compiled wheel paths. */
    public float[] pathBaseRatio = new float[0];
    public float[] highRatio = new float[0];
    public float[] lowRatio = new float[0];
    public float[] activeRatio = new float[0];
    public boolean[] lowMode = new boolean[0];

    public void allocate(int units) {
        unitCount = units;
        device = new int[units];
        deviceName = new String[units];
        pathBaseRatio = new float[units];
        highRatio = new float[units];
        lowRatio = new float[units];
        activeRatio = new float[units];
        lowMode = new boolean[units];
    }

    public void clear() {
        allocate(0);
    }
}
