package me.mzy.beamcraft.client.physics.electrics;

/** Read-only electric signal values consumed by one physics substep. */
public interface ElectricValues {
    double get(int signalId);
}
