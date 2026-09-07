package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Per-engine classification of the compliant device immediately downstream of a
 * combustion engine. BeamNG calls this device category {@code clutchlike}; the concrete
 * device owns its constitutive state in a dedicated SoA container.
 */
public final class ClutchlikeContainer {
    public static final byte TYPE_FRICTION_CLUTCH = 1;
    public static final byte TYPE_TORQUE_CONVERTER = 2;

    public byte[] type = new byte[0];
    public int[] device = new int[0];

    public void allocate(int units) {
        type = new byte[units];
        device = new int[units];
    }

    public void clear() {
        allocate(0);
    }
}
