/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Per-engine classification of the compliant device immediately downstream of a
 * combustion engine. BeamNG calls this device category {@code clutchlike}; the concrete
 * device owns its constitutive state in a dedicated SoA container.
 */
public final class ClutchlikeContainer {
    public static final byte TYPE_FRICTION_CLUTCH = 1;
    public static final byte TYPE_TORQUE_CONVERTER = 2;
    public static final byte TYPE_DCT_GEARBOX = 3;

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
