/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/** Runtime SoA for dual-clutch gearboxes, indexed by engine unit. */
public final class DctGearboxContainer {
    public int[] device = new int[0];
    public int[] gearIndex1 = new int[0];
    public int[] gearIndex2 = new int[0];
    public int[] shiftTarget = new int[0];
    public byte[] primaryClutch = new byte[0];
    public byte[] targetClutch = new byte[0];
    public float[] engagement1 = new float[0];
    public float[] engagement2 = new float[0];
    public float[] clutchCapacity = new float[0];
    public float[] clutchSpring = new float[0];
    public float[] clutchDampingRatio1 = new float[0];
    public float[] clutchDampingRatio2 = new float[0];
    public float[] clutchAngle1 = new float[0];
    public float[] clutchAngle2 = new float[0];
    public float[] clutchTorque1 = new float[0];
    public float[] clutchTorque2 = new float[0];

    public void allocate(int units) {
        device = new int[units];
        gearIndex1 = new int[units];
        gearIndex2 = new int[units];
        shiftTarget = new int[units];
        primaryClutch = new byte[units];
        targetClutch = new byte[units];
        engagement1 = new float[units];
        engagement2 = new float[units];
        clutchCapacity = new float[units];
        clutchSpring = new float[units];
        clutchDampingRatio1 = new float[units];
        clutchDampingRatio2 = new float[units];
        clutchAngle1 = new float[units];
        clutchAngle2 = new float[units];
        clutchTorque1 = new float[units];
        clutchTorque2 = new float[units];
    }

    public void clear() {
        allocate(0);
    }
}
