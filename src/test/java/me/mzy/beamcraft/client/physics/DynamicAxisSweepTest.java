package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DynamicAxisSweepTest {
    @Test
    void vehicleSegmentWithoutSelfCollisionIsSkippedBeforeSapQuery() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        triangleVehicle.nodes.count = 1;
        triangleVehicle.nodes.collision[0] = true;

        SoftBodyVehicle otherVehicle = new SoftBodyVehicle(null);
        otherVehicle.nodes.count = 1;
        otherVehicle.nodes.collision[0] = true;

        DynamicAxisSweep sweep = new DynamicAxisSweep();
        sweep.insertNodes(triangleVehicle, 0.0);
        sweep.insertNodes(otherVehicle, 0.0);
        sweep.updateAndSort();

        SweepResultBuffer result = new SweepResultBuffer();
        int rawHits = sweep.queryCollisionNodesInAABB(
                -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
                triangleVehicle, 1, 2, 3, -1, result);

        assertEquals(1, rawHits);
        assertEquals(1, result.count);
        assertTrue(result.vehicles[0] == otherVehicle);
    }

    @Test
    void selfCollisionPartBoundsGateTheVehicleSegment() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.nodes.count = 2;
        vehicle.nodes.collision[0] = true;
        vehicle.nodes.partId[0] = 0;
        vehicle.nodes.collision[1] = true;
        vehicle.nodes.selfCollision[1] = true;
        vehicle.nodes.partId[1] = 1;
        vehicle.nodes.posX[1] = vehicle.nodes.prevPosX[1] = 100.0f;

        DynamicAxisSweep sweep = new DynamicAxisSweep();
        sweep.insertNodes(vehicle, 0.0);
        sweep.updateAndSort();

        SweepResultBuffer result = new SweepResultBuffer();
        int rawHits = sweep.queryCollisionNodesInAABB(
                -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
                vehicle, 2, 3, 4, -1, result);

        assertEquals(0, rawHits);
        assertEquals(0, result.count);
    }

    @Test
    void movingNodeIsFoundWhenItsSweptIntervalEntersTheQuery() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        SoftBodyVehicle movingVehicle = new SoftBodyVehicle(null);
        movingVehicle.nodes.count = 1;
        movingVehicle.nodes.collision[0] = true;
        movingVehicle.nodes.posX[0] = 0.0f;
        movingVehicle.nodes.velX[0] = 10.0f;

        DynamicAxisSweep sweep = new DynamicAxisSweep();
        sweep.insertNodes(movingVehicle, 1.0);
        sweep.updateAndSort();

        SweepResultBuffer result = new SweepResultBuffer();
        int rawHits = sweep.queryCollisionNodesInAABB(
                9.5, -0.1, -0.1, 10.5, 0.1, 0.1,
                triangleVehicle, 0, 1, 2, -1, result);

        assertEquals(1, rawHits);
        assertEquals(1, result.count);
        assertTrue(result.vehicles[0] == movingVehicle);
        assertEquals(0, result.nodeIds[0]);
    }

    @Test
    void previousToCurrentSweepIsRetainedWhenNodeIsAlreadyMovingAway() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        SoftBodyVehicle movingVehicle = new SoftBodyVehicle(null);
        movingVehicle.nodes.count = 1;
        movingVehicle.nodes.collision[0] = true;
        movingVehicle.nodes.prevPosX[0] = -1.0f;
        movingVehicle.nodes.posX[0] = 1.0f;
        movingVehicle.nodes.velX[0] = 10.0f;

        DynamicAxisSweep sweep = new DynamicAxisSweep();
        sweep.insertNodes(movingVehicle, 0.1);
        sweep.updateAndSort();

        SweepResultBuffer result = new SweepResultBuffer();
        int rawHits = sweep.queryCollisionNodesInAABB(
                -0.1, -0.1, -0.1, 0.1, 0.1, 0.1,
                triangleVehicle, 0, 1, 2, -1, result);

        assertEquals(1, rawHits);
        assertEquals(1, result.count);
    }

    @Test
    void independentlyMovingTriangleVertexExpandsCandidateBounds() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        triangleVehicle.nodes.count = 3;
        triangleVehicle.nodes.posY[1] = 1.0f;
        triangleVehicle.nodes.posZ[2] = 1.0f;
        triangleVehicle.nodes.velX[0] = 5.0f;
        triangleVehicle.triangles.count = 1;
        triangleVehicle.triangles.node1[0] = 0;
        triangleVehicle.triangles.node2[0] = 1;
        triangleVehicle.triangles.node3[0] = 2;
        triangleVehicle.triangles.collision[0] = true;
        triangleVehicle.triangles.partId[0] = -1;

        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        nodeVehicle.nodes.count = 1;
        nodeVehicle.nodes.collision[0] = true;
        nodeVehicle.nodes.posX[0] = 5.0f;

        DynamicAxisSweep sweep = new DynamicAxisSweep();
        sweep.insertNodes(nodeVehicle, 1.0);
        sweep.updateAndSort();
        SoftBodyCollisionManager collisionManager = new SoftBodyCollisionManager();
        CollisionPipeline pipeline = new CollisionPipeline(
                new VoxelSnapshot(), sweep, collisionManager);

        pipeline.generateCollisionCandidates(triangleVehicle, 1.0);

        assertEquals(1, collisionManager.contactCount.get(),
                "a vertex sweep must be retained even when average triangle velocity would miss it");
    }

    @Test
    void topologyFiltersAreAppliedBeforeWritingSweepResults() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        triangleVehicle.nodes.count = 6;
        triangleVehicle.matrixPartStride = 1;
        triangleVehicle.nodeInPartMatrix = new boolean[6];
        for (int node = 0; node < triangleVehicle.nodes.count; node++) {
            triangleVehicle.nodes.collision[node] = true;
            triangleVehicle.nodes.posX[node] = node * 0.01f;
        }
        triangleVehicle.nodes.selfCollision[0] = true;
        triangleVehicle.nodes.selfCollision[1] = true;
        triangleVehicle.nodes.selfCollision[2] = true;
        triangleVehicle.nodes.selfCollision[4] = true;
        triangleVehicle.nodes.selfCollision[5] = true;
        triangleVehicle.nodeInPartMatrix[4] = true;

        SoftBodyVehicle otherVehicle = new SoftBodyVehicle(null);
        otherVehicle.nodes.count = 1;
        otherVehicle.nodes.collision[0] = true;
        otherVehicle.nodes.posX[0] = 0.03f;

        DynamicAxisSweep sweep = new DynamicAxisSweep();
        sweep.insertNodes(triangleVehicle, 0.0);
        sweep.insertNodes(otherVehicle, 0.0);
        sweep.updateAndSort();

        SweepResultBuffer result = new SweepResultBuffer();
        int rawHits = sweep.queryCollisionNodesInAABB(
                -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
                triangleVehicle, 0, 1, 2, 0, result);

        assertEquals(6, rawHits,
                "the same-vehicle segment must contain only self-collidable nodes");
        assertEquals(2, result.count);
        boolean foundEligibleSelfNode = false;
        boolean foundOtherVehicleNode = false;
        for (int index = 0; index < result.count; index++) {
            foundEligibleSelfNode |= result.vehicles[index] == triangleVehicle && result.nodeIds[index] == 5;
            foundOtherVehicleNode |= result.vehicles[index] == otherVehicle && result.nodeIds[index] == 0;
        }
        assertTrue(foundEligibleSelfNode);
        assertTrue(foundOtherVehicleNode);

        result.clear();
        rawHits = sweep.queryCollisionNodesInAABB(
                -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
                triangleVehicle, 0, 1, 2, -1, result);
        assertEquals(6, rawHits);
        assertEquals(3, result.count,
                "an invalid triangle part must not apply the part-matrix filter");
    }
}
