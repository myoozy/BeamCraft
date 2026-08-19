package me.mzy.beamcraft.texture;

import org.junit.jupiter.api.Test;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

class TextureOwnershipTest {

    @Test
    void commonSourcesResolveToSharedNull() {
        // Common sources use the durable null ownership marker, never a magic
        // "common" namespace (which could collide with a real namespace or be
        // released by accident). Null entries are never reclaimed by a
        // namespace release.
        Set<String> common = Set.of("C:/vehicles/common.zip", "C:/vehicles/common");
        assertNull(TextureOwnership.resolve("C:/vehicles/common.zip", common, "car", Set.of("C:/vehicles/car.zip")));
        assertNull(TextureOwnership.resolve("C:/vehicles/common", common, "car", Set.of("C:/vehicles/car.zip")));
    }

    @Test
    void ownSourcesResolveToNamespace() {
        assertEquals("car",
                TextureOwnership.resolve("C:/vehicles/car.zip", Set.of("C:/vehicles/common.zip"), "car",
                        Set.of("C:/vehicles/car.zip")));
    }

    @Test
    void foreignOrUnknownSourcesAreShared() {
        // A source owned by another namespace is never evicted by ours.
        assertNull(TextureOwnership.resolve("C:/vehicles/other.zip", Set.of("C:/vehicles/common.zip"), "car",
                Set.of("C:/vehicles/car.zip")));
        assertNull(TextureOwnership.resolve("C:/vehicles/mystery.zip", Set.of("C:/vehicles/common.zip"), "car",
                Set.of("C:/vehicles/car.zip")));
    }

    @Test
    void nullInputsAreShared() {
        assertNull(TextureOwnership.resolve(null, Set.of(), "car", Set.of()));
        assertNull(TextureOwnership.resolve("C:/x.zip", null, null, null));
        assertNull(TextureOwnership.resolve("C:/x.zip", Set.of(), null, null));
    }

    @Test
    void commonWinsOverNamespaceFallback() {
        // Even when the caller is a vehicle, a common source stays shared
        // (null) rather than being owned by the vehicle namespace.
        assertNull(TextureOwnership.resolve("C:/vehicles/common.zip", Set.of("C:/vehicles/common.zip"), "car",
                Set.of("C:/vehicles/common.zip")));
    }
}
