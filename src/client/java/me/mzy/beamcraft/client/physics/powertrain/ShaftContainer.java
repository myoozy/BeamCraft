/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Compiled shaft parameters, one row per {@code shaft} device. Build-time only: the
 * gear ratio is folded into {@link PowertrainTopologyContainer#deviceRatio} and the connected
 * wheel name into {@link DrivenWheelPathContainer} by {@link PowertrainCompiler}.
 */
public final class ShaftContainer {
    public int count;
    public int[] device = new int[0];
    public float[] gearRatio = new float[0];
    public String[] connectedWheel = new String[0];
    public float[] friction = new float[0];
    public float[] dynamicFriction = new float[0];
    public float[] torqueLossCoef = new float[0];

    public void allocate(int size) {
        count = size;
        device = new int[size];
        gearRatio = new float[size];
        connectedWheel = new String[size];
        friction = new float[size];
        dynamicFriction = new float[size];
        torqueLossCoef = new float[size];
    }

    public void clear() {
        allocate(0);
    }
}
