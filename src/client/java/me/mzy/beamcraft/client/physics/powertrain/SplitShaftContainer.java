/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Runtime SoA for every compiled split shaft and its two rigid downstream path sets.
 * Device rows and path rows are packed independently; {@code unitStart}/{@code unitCount}
 * map an engine unit to a contiguous range of split devices without runtime allocation.
 */
public final class SplitShaftContainer {
    public static final byte MODE_DISCONNECTED = 0;
    public static final byte MODE_LOCKED = 1;
    public static final byte MODE_VISCOUS = 2;

    public int count;
    public int[] unitStart = new int[0];
    public short[] unitCount = new short[0];
    public int[] unit = new int[0];
    public int[] device = new int[0];
    public String[] deviceName = new String[0];
    public byte[] configuredMode = new byte[0];
    public byte[] initialMode = new byte[0];
    public byte[] activeMode = new byte[0];
    public int[] primaryOutputID = new int[0];
    public boolean[] canDisconnect = new boolean[0];
    public float[] clutchRatio = new float[0];
    public float[] defaultClutchRatio = new float[0];
    public float[] lockCapacity = new float[0];
    public float[] lockSpring = new float[0];
    public float[] lockDampingRatio = new float[0];
    public float[] lockTorque = new float[0];
    public float[] shaftAngle = new float[0];
    public float[] viscousCoef = new float[0];
    public float[] viscousCapacity = new float[0];
    public float[] viscousExponent = new float[0];
    public float[] viscousSmoothing = new float[0];
    public float[] viscousTorque = new float[0];

    public int[] primaryPathStart = new int[0];
    public short[] primaryPathCount = new short[0];
    public int[] secondaryPathStart = new int[0];
    public short[] secondaryPathCount = new short[0];
    public int[] pathWheel = new int[0];
    public float[] pathGain = new float[0];
    public byte[] pathFlags = new byte[0];

    public void allocate(int units, int devices, int paths) {
        count = devices;
        unitStart = new int[units];
        unitCount = new short[units];
        unit = new int[devices];
        device = new int[devices];
        deviceName = new String[devices];
        configuredMode = new byte[devices];
        initialMode = new byte[devices];
        activeMode = new byte[devices];
        primaryOutputID = new int[devices];
        canDisconnect = new boolean[devices];
        clutchRatio = new float[devices];
        defaultClutchRatio = new float[devices];
        lockCapacity = new float[devices];
        lockSpring = new float[devices];
        lockDampingRatio = new float[devices];
        lockTorque = new float[devices];
        shaftAngle = new float[devices];
        viscousCoef = new float[devices];
        viscousCapacity = new float[devices];
        viscousExponent = new float[devices];
        viscousSmoothing = new float[devices];
        viscousTorque = new float[devices];
        primaryPathStart = new int[devices];
        primaryPathCount = new short[devices];
        secondaryPathStart = new int[devices];
        secondaryPathCount = new short[devices];
        pathWheel = new int[paths];
        pathGain = new float[paths];
        pathFlags = new byte[paths];
    }

    public void clear() {
        allocate(0, 0, 0);
    }
}
