package me.mzy.beamcraft.client.physics.powertrain;

/** Runtime state for one optional two-output split shaft per engine unit. */
public final class SplitShaftContainer {
    public static final byte MODE_DISCONNECTED = 0;
    public static final byte MODE_LOCKED = 1;
    public static final byte MODE_VISCOUS = 2;

    public int unitCount;
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

    public void allocate(int units) {
        unitCount = units;
        device = new int[units];
        deviceName = new String[units];
        configuredMode = new byte[units];
        initialMode = new byte[units];
        activeMode = new byte[units];
        primaryOutputID = new int[units];
        canDisconnect = new boolean[units];
        clutchRatio = new float[units];
        defaultClutchRatio = new float[units];
        lockCapacity = new float[units];
        lockSpring = new float[units];
        lockDampingRatio = new float[units];
        lockTorque = new float[units];
        shaftAngle = new float[units];
        viscousCoef = new float[units];
        viscousCapacity = new float[units];
        viscousExponent = new float[units];
        viscousSmoothing = new float[units];
        viscousTorque = new float[units];
    }

    public void clear() {
        allocate(0);
    }
}
