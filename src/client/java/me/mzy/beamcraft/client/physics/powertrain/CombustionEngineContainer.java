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
    /** BeamNG-compatible proportional idle controller parameters. */
    public float[] idleControllerP = new float[0];
    public float[] maxIdleThrottle = new float[0];
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
    public float[] limiterCutRemaining = new float[0];

    /** Allocates the per-unit rows and flattened torque-curve storage. */
    public void allocate(int units, int curvePoints) {
        unitCount = units;
        engineDevice = new int[units];
        clutchDevice = new int[units];
        engineInertia = new float[units];
        engineAV = new float[units];
        idleAV = new float[units];
        engineFriction = new float[units];
        engineDynamicFriction = new float[units];
        engineBrakeTorque = new float[units];
        curveStart = new int[units];
        curveCount = new short[units];
        curveRPM = new float[curvePoints];
        curveTorque = new float[curvePoints];
        starterTorque = new float[units];
        starterMaxAV = new float[units];
        crankingAV = new float[units];
        idleLossThrottle = new float[units];
        idleControllerP = new float[units];
        maxIdleThrottle = new float[units];
        playerThrottle = new float[units];
        actualThrottle = new float[units];
        revLimiterRPM = new float[units];
        revLimiterType = new byte[units];
        revLimiterCutTime = new float[units];
        revLimiterMaxRPMDrop = new float[units];
        sparkEnabled = new boolean[units];
        fuelEnabled = new boolean[units];
        starterActive = new boolean[units];
        limiterCutRemaining = new float[units];
    }

    /** Resets every array so the container holds zero units. */
    public void clear() {
        allocate(0, 0);
    }
}
