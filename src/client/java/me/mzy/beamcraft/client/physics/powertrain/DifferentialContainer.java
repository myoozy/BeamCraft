/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/** Runtime SoA for two-output differentials and their reflected wheel domains. */
public final class DifferentialContainer {
    public static final byte MODE_OPEN = 0;
    public static final byte MODE_LSD = 1;
    public static final byte MODE_VISCOUS = 2;
    public static final byte MODE_LOCKED = 3;
    public static final byte MODE_ACTIVE_LOCK = 4;

    public int count;
    public int[] unitStart = new int[0];
    public short[] unitCount = new short[0];
    public int[] unit = new int[0];
    public int[] device = new int[0];
    public String[] deviceName = new String[0];
    public float[] gearRatio = new float[0];
    public float[] diffTorqueSplit = new float[0];
    public String[] diffType = new String[0];
    public int[] availableModes = new int[0];
    public byte[] initialMode = new byte[0];
    public byte[] activeMode = new byte[0];

    public float[] friction = new float[0];
    public float[] dynamicFriction = new float[0];
    public float[] torqueLossCoef = new float[0];
    public float[] lsdPreload = new float[0];
    public float[] lsdLockCoef = new float[0];
    public float[] lsdRevLockCoef = new float[0];
    public float[] viscousCoef = new float[0];
    public float[] viscousCapacity = new float[0];
    public float[] viscousExponent = new float[0];
    public float[] viscousSmoothing = new float[0];
    public float[] lockCapacity = new float[0];
    public float[] lockSpring = new float[0];
    public float[] lockDampingRatio = new float[0];
    public float[] activeLockCapacity = new float[0];
    public float[] activeLockCoef = new float[0];

    public float[] inputTorque = new float[0];
    public float[] lockTorque = new float[0];
    public float[] diffAngle = new float[0];
    public float[] viscousTorque = new float[0];

    public int[] output1PathStart = new int[0];
    public short[] output1PathCount = new short[0];
    public int[] output2PathStart = new int[0];
    public short[] output2PathCount = new short[0];
    public int[] pathWheel = new int[0];
    public float[] pathGain = new float[0];
    public byte[] pathFlags = new byte[0];

    public int[] inputTermStart = new int[0];
    public short[] inputTermCount = new short[0];
    /** -1 selects the unit's clutch output; otherwise this indexes a split shaft. */
    public int[] termSplit = new int[0];
    public float[] termGain = new float[0];
    public byte[] termFlags = new byte[0];

    public void allocate(int units, int devices, int paths, int terms) {
        count = devices;
        unitStart = new int[units];
        unitCount = new short[units];
        unit = new int[devices];
        device = new int[devices];
        deviceName = new String[devices];
        gearRatio = new float[devices];
        diffTorqueSplit = new float[devices];
        diffType = new String[devices];
        availableModes = new int[devices];
        initialMode = new byte[devices];
        activeMode = new byte[devices];
        friction = new float[devices];
        dynamicFriction = new float[devices];
        torqueLossCoef = new float[devices];
        lsdPreload = new float[devices];
        lsdLockCoef = new float[devices];
        lsdRevLockCoef = new float[devices];
        viscousCoef = new float[devices];
        viscousCapacity = new float[devices];
        viscousExponent = new float[devices];
        viscousSmoothing = new float[devices];
        lockCapacity = new float[devices];
        lockSpring = new float[devices];
        lockDampingRatio = new float[devices];
        activeLockCapacity = new float[devices];
        activeLockCoef = new float[devices];
        inputTorque = new float[devices];
        lockTorque = new float[devices];
        diffAngle = new float[devices];
        viscousTorque = new float[devices];
        output1PathStart = new int[devices];
        output1PathCount = new short[devices];
        output2PathStart = new int[devices];
        output2PathCount = new short[devices];
        pathWheel = new int[paths];
        pathGain = new float[paths];
        pathFlags = new byte[paths];
        inputTermStart = new int[devices];
        inputTermCount = new short[devices];
        termSplit = new int[terms];
        termGain = new float[terms];
        termFlags = new byte[terms];
    }

    public void clear() {
        allocate(0, 0, 0, 0);
    }
}
