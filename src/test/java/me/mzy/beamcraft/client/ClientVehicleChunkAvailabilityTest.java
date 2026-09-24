package me.mzy.beamcraft.client;

import me.mzy.beamcraft.client.physics.NodeContainer;
import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ClientVehicleChunkAvailabilityTest {

    @Test
    void requiresNeighbouringChunkWhenSafetyMarginCrossesBoundary() {
        NodeContainer nodes = nodeAt(15.0f, 8.0f);
        Set<Long> loaded = Set.of(key(0, 0), key(1, 0));

        assertTrue(ClientVehicleManager.hasRequiredChunks(
                nodes, 0.0, 0.0, 0.05, (x, z) -> loaded.contains(key(x, z))));
        assertFalse(ClientVehicleManager.hasRequiredChunks(
                nodes, 0.0, 0.0, 0.05, (x, z) -> x == 0 && z == 0));
    }

    @Test
    void checksPredictedNodePositionBeforeAllowingPhysics() {
        NodeContainer nodes = nodeAt(8.0f, 8.0f);
        nodes.velX[0] = 200.0f;
        Set<Long> requested = new HashSet<>();

        boolean available = ClientVehicleManager.hasRequiredChunks(
                nodes, 0.0, 0.0, 0.05, (x, z) -> {
                    requested.add(key(x, z));
                    return x == 0 && z == 0;
                });

        assertFalse(available);
        assertTrue(requested.contains(key(1, 0)));
    }

    @Test
    void usesFloorSemanticsForNegativeWorldCoordinates() {
        NodeContainer nodes = nodeAt(-0.25f, 8.0f);

        assertTrue(ClientVehicleManager.hasRequiredChunks(
                nodes, 0.0, 0.0, 0.05, (x, z) -> (x == -1 || x == 0) && z == 0));
    }

    private static NodeContainer nodeAt(float x, float z) {
        NodeContainer nodes = new NodeContainer();
        nodes.count = 1;
        nodes.posX[0] = x;
        nodes.posZ[0] = z;
        return nodes;
    }

    private static long key(int x, int z) {
        return ((long) x << 32) ^ (z & 0xffffffffL);
    }
}
