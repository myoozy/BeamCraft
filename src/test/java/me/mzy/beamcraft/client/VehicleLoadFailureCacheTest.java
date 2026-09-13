package me.mzy.beamcraft.client;

import org.junit.jupiter.api.Test;

import java.util.UUID;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VehicleLoadFailureCacheTest {

    @Test
    void suppressesOnlyTheSameFailedSetup() {
        VehicleLoadFailureCache cache = new VehicleLoadFailureCache();
        UUID entityUuid = UUID.randomUUID();

        assertTrue(cache.shouldAttempt(6, entityUuid, "etk800", "846x_ttsport_plus_DCT.pc"));
        cache.recordFailure(6, entityUuid, "etk800", "846x_ttsport_plus_DCT.pc");

        assertFalse(cache.shouldAttempt(6, entityUuid, "etk800", "846x_ttsport_plus_DCT.pc"));
        assertTrue(cache.shouldAttempt(6, entityUuid, "etk800", "another.pc"));
        assertTrue(cache.shouldAttempt(6, UUID.randomUUID(), "etk800", "846x_ttsport_plus_DCT.pc"));
    }

    @Test
    void clearingWorldAllowsRetry() {
        VehicleLoadFailureCache cache = new VehicleLoadFailureCache();
        UUID entityUuid = UUID.randomUUID();
        cache.recordFailure(6, entityUuid, "etk800", "broken.pc");

        cache.clear();

        assertTrue(cache.shouldAttempt(6, entityUuid, "etk800", "broken.pc"));
    }
}
