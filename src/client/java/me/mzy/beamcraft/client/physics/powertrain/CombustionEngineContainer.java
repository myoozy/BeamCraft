package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Per-unit combustion engine + torque-curve SoA. One row per compiled engine→clutch
 * unit; the torque curve is stored flat in {@code curveRPM}/{@code curveTorque} and
 * indexed by {@code curveStart}/{@code curveCount}.
 *
 * <p>{@code engineAV}, {@code clutchAngle} and {@code clutchTorque} live in this and
 * the clutch container respectively and are the only per-unit state the substep mutates.
 *
 * <p>Since the powertrain stage this container also carries the starter, rev-limiter and
 * idle-controller runtime state. Combustion is an aggregate actuation state: it only
 * produces torque while {@code sparkEnabled && fuelEnabled} (both cut by the rev limiter)
 * and {@code engineAV >= crankingAV}.
 */
public final class CombustionEngineContainer {
    public static final byte LIMITER_TYPE_TIME = 0;
    public static final byte LIMITER_TYPE_SOFT = 1;

    public int unitCount;
    public int[] engineDevice = new int[0];
    public int[] clutchDevice = new int[0];
    public float[] engineInertia = new float[0];
    public float[] engineAV = new float[0];
    public float[] idleAV = new float[0];
    public float[] engineFriction = new float[0];
    public float[] engineDynamicFriction = new float[0];
    public float[] engineBrakeTorque = new float[0];
    public int[] curveStart = new int[0];
    public short[] curveCount = new short[0];
    public float[] curveRPM = new float[0];
    public float[] curveTorque = new float[0];

    // Starter: external crank torque while starterActive and engineAV < starterMaxAV.
    public float[] starterTorque = new float[0];
    public float[] starterMaxAV = new float[0];
    // Minimum crank speed (rad/s) at which combustion is allowed.
    public float[] crankingAV = new float[0];
    // Closed-throttle/top-screw idle feedforward (0..1 throttle) that covers idle losses.
    public float[] idleLossThrottle = new float[0];
    /** Player pedal command and physical throttle opening are deliberately separate. */
    public float[] playerThrottle = new float[0];
    public float[] actualThrottle = new float[0];

    // Rev limiter configuration.
    public float[] revLimiterRPM = new float[0];
    public byte[] revLimiterType = new byte[0];
    public float[] revLimiterCutTime = new float[0];
    public float[] revLimiterMaxRPMDrop = new float[0];

    // Rev limiter / combustion runtime state.
    public boolean[] sparkEnabled = new boolean[0];
    public boolean[] fuelEnabled = new boolean[0];
    public boolean[] starterActive = new boolean[0];
    public float[] idleIntegral = new float[0];
    public float[] limiterCutRemaining = new float[0];

    /** Resets every array so the container holds zero units. */
    public void clear() {
        unitCount = 0;
        engineDevice = new int[0];
        clutchDevice = new int[0];
        engineInertia = new float[0];
        engineAV = new float[0];
        idleAV = new float[0];
        engineFriction = new float[0];
        engineDynamicFriction = new float[0];
        engineBrakeTorque = new float[0];
        curveStart = new int[0];
        curveCount = new short[0];
        curveRPM = new float[0];
        curveTorque = new float[0];
        starterTorque = new float[0];
        starterMaxAV = new float[0];
        crankingAV = new float[0];
        idleLossThrottle = new float[0];
        playerThrottle = new float[0];
        actualThrottle = new float[0];
        revLimiterRPM = new float[0];
        revLimiterType = new byte[0];
        revLimiterCutTime = new float[0];
        revLimiterMaxRPMDrop = new float[0];
        sparkEnabled = new boolean[0];
        fuelEnabled = new boolean[0];
        starterActive = new boolean[0];
        idleIntegral = new float[0];
        limiterCutRemaining = new float[0];
    }
}
