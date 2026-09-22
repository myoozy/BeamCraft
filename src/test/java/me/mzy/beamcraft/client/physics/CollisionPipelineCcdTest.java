package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CollisionPipelineCcdTest {
    @Test
    void resolvesNodeThatCrossedTriangleAndEndedOutsideCurrentAabb() {
        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        nodeVehicle.nodes.count = 1;
        nodeVehicle.nodes.mass[0] = 1.0f;
        nodeVehicle.nodes.prevPosX[0] = 0.25f;
        nodeVehicle.nodes.prevPosY[0] = 0.25f;
        nodeVehicle.nodes.prevPosZ[0] = 1.0f;
        nodeVehicle.nodes.posX[0] = 0.25f;
        nodeVehicle.nodes.posY[0] = 0.25f;
        nodeVehicle.nodes.posZ[0] = -1.0f;

        SoftBodyVehicle triangleVehicle = stationaryUnitTriangle();
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        manager.addContact(nodeVehicle, 0, triangleVehicle, 0, 1, 2);
        manager.buildAndColorBatches();

        CollisionPipeline pipeline = new CollisionPipeline(
                new VoxelSnapshot(), new DynamicAxisSweep(), manager);
        float before = nodeVehicle.nodes.posZ[0];
        pipeline.solveSoftBodyContacts(0.0005f);

        assertEquals(1, manager.sweptResolvedCount.get());
        assertTrue(nodeVehicle.nodes.posZ[0] > before);
    }

    private static SoftBodyVehicle stationaryUnitTriangle() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.nodes.count = 3;
        vehicle.nodes.posX[1] = vehicle.nodes.prevPosX[1] = 1.0f;
        vehicle.nodes.posY[2] = vehicle.nodes.prevPosY[2] = 1.0f;
        for (int node = 0; node < 3; node++) {
            vehicle.nodes.mass[node] = 1.0f;
        }
        return vehicle;
    }
}
