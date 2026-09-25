/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see
 * LICENSES/bCDDL-1.1.txt.
 *
 * Adapted from BeamNG.drive lua/vehicle/controller/drivingDynamics/actuators/
 * adaptiveDampers.lua. Java adaptation and modifications contributed by
 * M1AO and BeamCraft contributors.
 */
package me.mzy.beamcraft.client.physics;

import java.util.List;

/**
 * BeamNG-compatible beam selection and coefficient application for adaptive
 * damper modes. Threading, controller lifecycle and command dispatch remain in
 * {@link AdaptiveDamperActuators}.
 */
final class AdaptiveDamperCompatibility {
    private AdaptiveDamperCompatibility() {
    }

    static int[] resolveBeamIndices(BoundedBeamContainer beams, List<String> beamNames) {
        if (beamNames == null || beamNames.isEmpty()) return new int[0];
        int[] collected = new int[beamNames.size()];
        int size = 0;
        for (String beamName : beamNames) {
            for (int index : beams.indicesForName(beamName)) {
                boolean duplicate = false;
                for (int i = 0; i < size; i++) {
                    if (collected[i] == index) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate) continue;
                if (size == collected.length) collected = java.util.Arrays.copyOf(collected, size * 2);
                collected[size++] = index;
            }
        }
        return java.util.Arrays.copyOf(collected, size);
    }

    static void applyMode(BoundedBeamContainer beams, int[] beamIndices, AdaptiveDamperMode mode) {
        int count = beams.count;
        for (int index : beamIndices) {
            if (index < 0 || index >= count) continue;

            float ceiling = beams.dampStabilityCeiling[index];
            beams.damp[index] = scaled(beams.authoredDamp[index], mode.beamDampCoef(), ceiling);
            beams.dampFast[index] = scaled(
                    beams.authoredDampFast[index], mode.beamDampFastCoef(), ceiling);
            beams.dampRebound[index] = scaled(
                    beams.authoredDampRebound[index], mode.beamDampReboundCoef(), ceiling);
            beams.dampReboundFast[index] = scaled(
                    beams.authoredDampReboundFast[index], mode.beamDampReboundFastCoef(), ceiling);

            // The compatible controller uses the same scaled split for compression
            // and rebound, overriding any separately authored rebound split.
            float split = scaledSplit(beams.authoredDampVelocitySplit[index],
                    mode.beamDampVelocitySplitCoef());
            beams.dampVelocitySplit[index] = split;
            beams.dampVelocitySplitRebound[index] = split;
        }
    }

    private static float scaled(float authored, float coefficient, float ceiling) {
        float value = authored * coefficient;
        if (Float.isNaN(value)) return ceiling;
        return Math.min(value, ceiling);
    }

    private static float scaledSplit(float authored, float coefficient) {
        float value = authored * coefficient;
        if (Float.isNaN(value)) return authored;
        return Math.min(value, Float.MAX_VALUE);
    }
}
