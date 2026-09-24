package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Compiled direct-drive electric motors and their rigid wheel paths.
 *
 * <p>The motor has no independent rotational state in this MVP. Its angular velocity is
 * reconstructed from the downstream wheel domain and its curve torque is mapped directly
 * back to those wheels. This is the rigid/direct-drive limit, not a permanently locked clutch.
 */
public final class ElectricMotorContainer {
    public int motorCount;
    public int[] device = new int[0];
    public float[] motorAV = new float[0];
    public float[] outputTorque = new float[0];
    public float[] maxRegenTorque = new float[0];
    public float[] maxRegenPowerW = new float[0];
    public float[] onePedalRegenCoef = new float[0];
    public int[] curveStart = new int[0];
    public short[] curveCount = new short[0];
    public float[] curveRPM = new float[0];
    public float[] curveTorque = new float[0];
    public int[] pathStart = new int[0];
    public short[] pathCount = new short[0];
    public int[] pathWheel = new int[0];
    public float[] pathGain = new float[0];

    public void allocate(int motors, int curvePoints, int paths) {
        motorCount = motors;
        device = new int[motors];
        motorAV = new float[motors];
        outputTorque = new float[motors];
        maxRegenTorque = new float[motors];
        maxRegenPowerW = new float[motors];
        onePedalRegenCoef = new float[motors];
        curveStart = new int[motors];
        curveCount = new short[motors];
        curveRPM = new float[curvePoints];
        curveTorque = new float[curvePoints];
        pathStart = new int[motors];
        pathCount = new short[motors];
        pathWheel = new int[paths];
        pathGain = new float[paths];
    }

    public void clear() {
        allocate(0, 0, 0);
    }
}
