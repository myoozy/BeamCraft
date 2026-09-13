package me.mzy.beamcraft.client;

import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

/** Prevents a permanently invalid vehicle setup from being rebuilt every client tick. */
final class VehicleLoadFailureCache {
    private final Map<Integer, LoadKey> failures = new HashMap<>();

    boolean shouldAttempt(int entityId, UUID entityUuid, String rootPart, String pcFile) {
        return !new LoadKey(entityUuid, rootPart, pcFile).equals(failures.get(entityId));
    }

    void recordFailure(int entityId, UUID entityUuid, String rootPart, String pcFile) {
        failures.put(entityId, new LoadKey(entityUuid, rootPart, pcFile));
    }

    void recordSuccess(int entityId) {
        failures.remove(entityId);
    }

    void removeStale(int entityId, UUID currentEntityUuid) {
        LoadKey failed = failures.get(entityId);
        if (failed != null && !failed.entityUuid.equals(currentEntityUuid)) {
            failures.remove(entityId);
        }
    }

    void clear() {
        failures.clear();
    }

    private record LoadKey(UUID entityUuid, String rootPart, String pcFile) {
    }
}
