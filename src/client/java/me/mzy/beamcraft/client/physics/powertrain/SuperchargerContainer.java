/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/** Per-engine BeamNG-style mechanically driven supercharger configuration and runtime state. */
public final class SuperchargerContainer {
    public boolean[] existing = new boolean[0];
    public float[] gearRatio = new float[0];
    public float[] maxBlowerRPM = new float[0];
    public float[] pressurePSIPerRPM = new float[0];
    public float[] crankLossPerRPM = new float[0];
    public float[] pressureRatePa = new float[0];
    public float[] clutchEngageRPM = new float[0];
    public float[] clutchEngageRange = new float[0];
    public float[] clutchDisengageRPM = new float[0];
    public float[] clutchDisengageRange = new float[0];
    public float[] efficiencyB1 = new float[0];
    public float[] efficiencyB2 = new float[0];
    public float[] efficiencyB3 = new float[0];
    public float[] pulseFloor = new float[0];
    public float[] pulseLobes = new float[0];
    public int[] controllerStart = new int[0];
    public short[] controllerCount = new short[0];
    public float[] controllerThrottle = new float[0];
    public float[] controllerFactor = new float[0];

    public float[] blowerRPM = new float[0];
    public float[] pressurePa = new float[0];
    public float[] rawPressurePa = new float[0];
    public float[] lostTorqueCoef = new float[0];
    public float[] pulsePhase = new float[0];

    public void allocate(int units, int controllerPoints) {
        existing = new boolean[units];
        gearRatio = new float[units];
        maxBlowerRPM = new float[units];
        pressurePSIPerRPM = new float[units];
        crankLossPerRPM = new float[units];
        pressureRatePa = new float[units];
        clutchEngageRPM = new float[units];
        clutchEngageRange = new float[units];
        clutchDisengageRPM = new float[units];
        clutchDisengageRange = new float[units];
        efficiencyB1 = new float[units];
        efficiencyB2 = new float[units];
        efficiencyB3 = new float[units];
        pulseFloor = new float[units];
        pulseLobes = new float[units];
        controllerStart = new int[units];
        controllerCount = new short[units];
        controllerThrottle = new float[controllerPoints];
        controllerFactor = new float[controllerPoints];
        blowerRPM = new float[units];
        pressurePa = new float[units];
        rawPressurePa = new float[units];
        lostTorqueCoef = new float[units];
        pulsePhase = new float[units];
    }

    public void clear() {
        allocate(0, 0);
    }
}
