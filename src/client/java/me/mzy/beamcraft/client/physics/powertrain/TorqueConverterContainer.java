/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/** Per-engine torque-converter configuration and hot-loop state. */
public final class TorqueConverterContainer {
    public float[] couplingAVRatio = new float[0];
    public float[] stallTorqueRatio = new float[0];
    public float[] converterStiffness = new float[0];
    public float[] converterDiameter = new float[0];
    public float[] converterTorqueLimit = new float[0];
    public float[] additionalEngineInertia = new float[0];

    public int[] lockupSignalId = new int[0];
    public float[] lockupCapacity = new float[0];
    public float[] lockupSpring = new float[0];
    public float[] lockupDampingRatio = new float[0];
    public float[] lockupAngle = new float[0];
    public float[] lockupTorque = new float[0];

    public float[] speedRatio = new float[0];
    /** Reaction torque applied to the engine by the impeller plus lock-up clutch. */
    public float[] inputTorque = new float[0];
    /** Torque delivered to the gearbox by the turbine plus lock-up clutch. */
    public float[] outputTorque = new float[0];

    public void allocate(int units) {
        couplingAVRatio = new float[units];
        stallTorqueRatio = new float[units];
        converterStiffness = new float[units];
        converterDiameter = new float[units];
        converterTorqueLimit = new float[units];
        additionalEngineInertia = new float[units];
        lockupSignalId = new int[units];
        lockupCapacity = new float[units];
        lockupSpring = new float[units];
        lockupDampingRatio = new float[units];
        lockupAngle = new float[units];
        lockupTorque = new float[units];
        speedRatio = new float[units];
        inputTorque = new float[units];
        outputTorque = new float[units];
    }

    public void clear() {
        allocate(0);
    }
}
