package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Compiled device-graph (topology) in flat SoA form. One row per {@link PowertrainSpecs.DeviceSpec}
 * in active-part order; {@code children} holds the CSR-packed child lists indexed by
 * {@code childStart}/{@code childCount}.
 *
 * <p>This is pure build-time structure read by the solver only to resolve driven-wheel
 * paths during compilation; the substep itself never walks the graph.
 */
public final class PowertrainTopologyContainer {
    public static final byte TYPE_UNSUPPORTED = 0;
    public static final byte TYPE_ENGINE = 1;
    public static final byte TYPE_CLUTCH = 2;
    public static final byte TYPE_GEARBOX = 3;
    public static final byte TYPE_SHAFT = 4;
    public static final byte TYPE_DIFFERENTIAL = 5;
    public static final byte TYPE_TORSION_REACTOR = 6;

    public int deviceCount;
    public String[] deviceName = new String[0];
    public byte[] deviceType = new byte[0];
    public int[] parentDevice = new int[0];
    public int[] parentPort = new int[0];
    public int[] childStart = new int[0];
    public short[] childCount = new short[0];
    public int[] children = new int[0];
    public float[] deviceRatio = new float[0];

    public void allocateDevices(int devices) {
        deviceCount = devices;
        deviceName = new String[devices];
        deviceType = new byte[devices];
        parentDevice = new int[devices];
        parentPort = new int[devices];
        childStart = new int[devices];
        childCount = new short[devices];
        deviceRatio = new float[devices];
    }

    public void allocateChildren(int childLinks) {
        children = new int[childLinks];
    }

    /** Resets every array so the container holds zero devices. */
    public void clear() {
        allocateDevices(0);
        allocateChildren(0);
    }
}
