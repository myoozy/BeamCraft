package me.mzy.beamcraft.client.physics;

/**
 * One row of the BeamNG {@code adaptiveDampers} controller mode table.
 *
 * <p>The coefficients are dimensionless multipliers applied to the beam's
 * <em>authored</em> damping values; a missing column defaults to {@code 1}.
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
