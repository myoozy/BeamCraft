package me.mzy.beamcraft.client.physics;

import java.util.List;
import java.util.Map;

/**
 * Build-time description of one {@code adaptiveDampers} controller instance:
 * the JBeam {@code name} of the controller, the bounded beams it drives (by
 * their authored beam {@code name}) and its mode table.
 *
 * <p>Beam names are unresolved here; {@code SoftBodyVehicle} maps them onto
 * bounded-beam indices once every named beam exists.
 */
public record AdaptiveDamperSpec(
        String instanceName,
        List<String> dampBeamNames,
        Map<String, AdaptiveDamperMode> modes
) {}
