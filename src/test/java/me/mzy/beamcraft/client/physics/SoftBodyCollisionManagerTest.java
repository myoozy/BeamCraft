package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SoftBodyCollisionManagerTest {
    @Test
    void greedyColoringReusesLowestNonConflictingBatch() {
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        SoftBodyVehicle nodeVehicle = vehicleAtOffset(0);
        SoftBodyVehicle triangleVehicle = vehicleAtOffset(1_000);

        manager.addContact(nodeVehicle, 0, triangleVehicle, 0, 1, 2);
        manager.addContact(nodeVehicle, 1, triangleVehicle, 3, 4, 5);
        manager.addContact(nodeVehicle, 0, triangleVehicle, 6, 7, 8);

        manager.buildAndColorBatches();

        assertEquals(2, manager.activeBatchCount);
        assertEquals(2, manager.batchSize[0]);
        assertEquals(1, manager.batchSize[1]);
        assertEquals(0, manager.batches[0][0]);
        assertEquals(1, manager.batches[0][1]);
        assertEquals(2, manager.batches[1][0]);
        assertNormalBatchesHaveNoNodeConflicts(manager);
    }

    @Test
    void exhaustedColorsGoToDedicatedSerialOverflowBatch() {
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        SoftBodyVehicle nodeVehicle = vehicleAtOffset(0);
        SoftBodyVehicle triangleVehicle = vehicleAtOffset(1_000);

        int contactCount = 31;
        for (int contact = 0; contact < contactCount; contact++) {
            int triangleBase = contact * 3;
            manager.addContact(nodeVehicle, 0, triangleVehicle,
                    triangleBase, triangleBase + 1, triangleBase + 2);
        }

        manager.buildAndColorBatches();

        assertEquals(SoftBodyCollisionManager.MAX_BATCHES, manager.activeBatchCount);
        for (int batch = 0; batch < SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX; batch++) {
            assertEquals(1, manager.batchSize[batch]);
        }
        assertEquals(contactCount - SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX,
                manager.batchSize[SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX]);
        assertEquals(SoftBodyCollisionManager.MAX_BATCHES - 1,
                manager.batches[SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX][0]);
        assertNormalBatchesHaveNoNodeConflicts(manager);

        assertTrue(CollisionPipeline.canSolveBatchInParallel(0, 1_024, 1_024));
        assertFalse(CollisionPipeline.canSolveBatchInParallel(
                SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX, Integer.MAX_VALUE, 1_024));
    }

    @Test
    void separationCertificateIgnoresCommonTranslationButObservesRelativeMotion() {
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        manager.addContact(vehicleAtOffset(0), 0, vehicleAtOffset(1_000), 0, 1, 2);
        manager.recordSeparationCertificate(0, 0.5f,
                0.0f, 0.0f, 2.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f);

        assertTrue(manager.separationCertificateStillValid(0,
                0.0f, 0.0f, 2.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f));
        assertFalse(manager.separationCertificateStillValid(0,
                0.0f, 0.0f, 1.4f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f));
    }

    @Test
    void pointTriangleDistanceHandlesFaceEdgeAndVertexRegions() {
        assertEquals(4.0f, CollisionPipeline.pointTriangleDistanceSquared(
                0.25f, 0.25f, 2.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f), 1e-6f);
        assertEquals(0.5f, CollisionPipeline.pointTriangleDistanceSquared(
                1.0f, 1.0f, 0.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f), 1e-6f);
        assertEquals(1.0f, CollisionPipeline.pointTriangleDistanceSquared(
                -1.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f), 1e-6f);
    }

    private static SoftBodyVehicle vehicleAtOffset(int globalNodeOffset) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.globalNodeOffset = globalNodeOffset;
        return vehicle;
    }

    private static void assertNormalBatchesHaveNoNodeConflicts(SoftBodyCollisionManager manager) {
        boolean[] usedNodes = new boolean[SoftBodyCollisionManager.MAX_GLOBAL_NODES];
        for (int batch = 0; batch < SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX; batch++) {
            java.util.Arrays.fill(usedNodes, false);
            for (int entry = 0; entry < manager.batchSize[batch]; entry++) {
                int contact = manager.batches[batch][entry];
                SoftBodyVehicle nodeVehicle = manager.contactNodeVeh[contact];
                SoftBodyVehicle triangleVehicle = manager.contactTriVeh[contact];
                assertUnusedThenMark(usedNodes,
                        nodeVehicle.globalNodeOffset + manager.contactNodeId[contact]);
                assertUnusedThenMark(usedNodes,
                        triangleVehicle.globalNodeOffset + manager.contactTriA[contact]);
                assertUnusedThenMark(usedNodes,
                        triangleVehicle.globalNodeOffset + manager.contactTriB[contact]);
                assertUnusedThenMark(usedNodes,
                        triangleVehicle.globalNodeOffset + manager.contactTriC[contact]);
            }
        }
    }

    private static void assertUnusedThenMark(boolean[] usedNodes, int node) {
        assertFalse(usedNodes[node], "node " + node + " appears twice in one normal batch");
        usedNodes[node] = true;
    }
}
