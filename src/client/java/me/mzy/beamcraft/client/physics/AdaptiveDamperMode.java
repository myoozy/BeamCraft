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

/**
 * One row of the BeamNG {@code adaptiveDampers} controller mode table.
 *
 * <p>The coefficients are dimensionless multipliers applied to the beam's
 * <em>authored</em> damping values; a missing column defaults to {@code 1}
 * exactly as {@code adaptiveDampers.lua} does
 * ({@code mode.beamDampCoef or 1}).
 */
public record AdaptiveDamperMode(
        String name,
        float beamDampCoef,
        float beamDampFastCoef,
        float beamDampReboundCoef,
        float beamDampReboundFastCoef,
        float beamDampVelocitySplitCoef
) {
    /** Coefficient used when the author omitted a column. */
    public static final float DEFAULT_COEF = 1.0f;
}
