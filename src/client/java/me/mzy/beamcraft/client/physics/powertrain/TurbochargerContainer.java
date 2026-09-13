/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/** Per-engine BeamNG-style turbo configuration and runtime state. */
public final class TurbochargerContainer {
    public boolean[] existing = new boolean[0];
    public float[] inertia = new float[0];
    public float[] maxAV = new float[0];
    public float[] wastegateStartPa = new float[0];
    public float[] wastegateLimitPa = new float[0];
    public float[] maxExhaustPower = new float[0];
    public float[] backPressureCoef = new float[0];
    public float[] frictionCoef = new float[0];
    public float[] pressureFallRatePa = new float[0];
    public float[] wastegateP = new float[0];
    public float[] wastegateI = new float[0];
    public float[] wastegateD = new float[0];
    public boolean[] bovEnabled = new boolean[0];
    public float[] bovOpenThreshold = new float[0];
    public float[] bovOpenChangeThreshold = new float[0];

    public int[] pressureStart = new int[0];
    public short[] pressureCount = new short[0];
    public float[] pressureRPM = new float[0];
    public float[] pressurePSI = new float[0];
    public int[] engineStart = new int[0];
    public short[] engineCount = new short[0];
    public float[] engineRPM = new float[0];
    public float[] efficiency = new float[0];
    public float[] exhaustFactor = new float[0];

    public float[] turboAV = new float[0];
    public float[] pressurePa = new float[0];
    public float[] rawPressurePa = new float[0];
    public float[] wastegateIntegral = new float[0];
    public float[] lastBoostError = new float[0];
    public float[] wastegateFactor = new float[0];
    public float[] lastCombustionOutput = new float[0];
    public boolean[] bovEngaged = new boolean[0];

    public void allocate(int units, int pressurePoints, int enginePoints) {
        existing = new boolean[units];
        inertia = new float[units];
        maxAV = new float[units];
        wastegateStartPa = new float[units];
        wastegateLimitPa = new float[units];
        maxExhaustPower = new float[units];
        backPressureCoef = new float[units];
        frictionCoef = new float[units];
        pressureFallRatePa = new float[units];
        wastegateP = new float[units];
        wastegateI = new float[units];
        wastegateD = new float[units];
        bovEnabled = new boolean[units];
        bovOpenThreshold = new float[units];
        bovOpenChangeThreshold = new float[units];
        pressureStart = new int[units];
        pressureCount = new short[units];
        pressureRPM = new float[pressurePoints];
        pressurePSI = new float[pressurePoints];
        engineStart = new int[units];
        engineCount = new short[units];
        engineRPM = new float[enginePoints];
        efficiency = new float[enginePoints];
        exhaustFactor = new float[enginePoints];
        turboAV = new float[units];
        pressurePa = new float[units];
        rawPressurePa = new float[units];
        wastegateIntegral = new float[units];
        lastBoostError = new float[units];
        wastegateFactor = new float[units];
        lastCombustionOutput = new float[units];
        bovEngaged = new boolean[units];
    }

    public void clear() {
        allocate(0, 0, 0);
    }
}
