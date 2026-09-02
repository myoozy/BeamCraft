package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Per-unit runtime gearbox SoA. One row per compiled engine→clutch unit, kept in the
 * same order as {@link CombustionEngineContainer}. The gear ratios are flattened into
 * {@code gearRatios} in original JBeam order (index 0 = reverse, index 1 = neutral,
 * 2+ = forward gears) and indexed by {@code gearStart}/{@code gearCount} (CSR).
 *
 * <p>The solver reads {@link #activeRatio} to apply the current ratio as a factor over
 * the compile-time first-gear path gains. A gearbox with no gearbox device on its path
 * compiles as an implicit single 1.0 ratio. During a shift (or in neutral) the active
 * ratio is 0, disconnecting the torque path, while the engine still integrates.
 *
 * <p>{@code shiftRemaining} counts down in seconds and is only ever decremented by
 * {@code PowertrainSystem.solve(dt)} — never by wall/game clock.
 */
public final class GearboxContainer {
    public int unitCount;
    /** Topology device index of the gearbox, or -1 for an implicit single-ratio box. */
    public int[] device = new int[0];
    public String[] deviceName = new String[0];
    public String[] gearboxType = new String[0];
    public int[] gearStart = new int[0];
    public short[] gearCount = new short[0];
    /** Flattened gear ratios in JBeam order (reverse, neutral, forward…). */
    public float[] gearRatios = new float[0];
    /** Gear selected when the vehicle is created or reset (normally neutral). */
    public int[] initialGearIndex = new int[0];
    public int[] currentGearIndex = new int[0];
    /** Target gear during a shift; -1 when idle. */
    public int[] pendingGearIndex = new int[0];
    /** Ratio currently applied to the driveline (0 while shifting or in neutral). */
    public float[] activeRatio = new float[0];
    /** First-forward ratio baked into path gains; denominator of the dynamic-ratio factor. */
    public float[] pathBaseRatio = new float[0];
    public float[] shiftRemaining = new float[0];
    public float[] shiftDuration = new float[0];
    public boolean[] fixedFirstGear = new boolean[0];

    public void allocate(int units, int ratioCount) {
        unitCount = units;
        device = new int[units];
        deviceName = new String[units];
        gearboxType = new String[units];
        gearStart = new int[units];
        gearCount = new short[units];
        gearRatios = new float[ratioCount];
        initialGearIndex = new int[units];
        currentGearIndex = new int[units];
        pendingGearIndex = new int[units];
        activeRatio = new float[units];
        pathBaseRatio = new float[units];
        shiftRemaining = new float[units];
        shiftDuration = new float[units];
        fixedFirstGear = new boolean[units];
    }

    /** Resets every array so the container holds zero units. */
    public void clear() {
        allocate(0, 0);
    }
}
