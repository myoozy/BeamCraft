package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CollisionChunkIndexTest {
    @Test
    void completeTriangleFindsNodeEvenWhenItSpansSeveralNodeChunks() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        setNode(triangleVehicle, 0, -10.0f, 0.0f, 0.0f, false);
        setNode(triangleVehicle, 1, 10.0f, 0.0f, 0.0f, false);
        setNode(triangleVehicle, 2, 0.0f, 1.0f, 0.0f, false);
        triangleVehicle.triangles.count = 1;
        triangleVehicle.triangles.node1[0] = 0;
        triangleVehicle.triangles.node2[0] = 1;
        triangleVehicle.triangles.node3[0] = 2;
        triangleVehicle.triangles.partId[0] = -1;
        triangleVehicle.triangles.collision[0] = true;

        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        for (int node = 0; node < 100; node++) {
            setNode(nodeVehicle, node, node == 50 ? 0.0f : node - 50.0f, node == 50 ? 0.5f : 100.0f, 0.0f, true);
        }

        triangleVehicle.collisionChunks.rebuild();
        nodeVehicle.collisionChunks.rebuild();
        triangleVehicle.collisionChunks.refit(0.0);
        nodeVehicle.collisionChunks.refit(0.0);
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        CollisionPipeline pipeline = new CollisionPipeline(new VoxelSnapshot(), new DynamicAxisSweep(), manager);

        pipeline.generateChunkCollisionCandidates(triangleVehicle, List.of(triangleVehicle, nodeVehicle));

        assertEquals(1, manager.contactCount.get());
        assertEquals(50, manager.contactNodeId[0]);
        assertTrue(nodeVehicle.collisionChunks.nodeChunkCount() >= 2);
    }

    @Test
    void spatiallyOversizedTriangleGetsItsOwnMeshlet() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        int node = 0;
        for (int triangle = 0; triangle < 12; triangle++) {
            float x = triangle * 0.2f;
            setNode(vehicle, node, x, 0.0f, 0.0f, false);
            setNode(vehicle, node + 1, x + 0.1f, 0.0f, 0.0f, false);
            setNode(vehicle, node + 2, x, 0.1f, 0.0f, false);
            addTriangle(vehicle, triangle, node, node + 1, node + 2);
            node += 3;
        }
        setNode(vehicle, node, -50.0f, 0.0f, 0.0f, false);
        setNode(vehicle, node + 1, 50.0f, 0.0f, 0.0f, false);
        setNode(vehicle, node + 2, 0.0f, 10.0f, 0.0f, false);
        addTriangle(vehicle, 12, node, node + 1, node + 2);

        vehicle.collisionChunks.rebuild();

        assertEquals(1, vehicle.collisionChunks.oversizedTriangleCount());
        assertEquals(1, vehicle.collisionChunks.triangleMeshletEnd(0)
                - vehicle.collisionChunks.triangleMeshletStart(0));
    }

    @Test
    void predictedNodeAndTriangleBoundsRetainFutureOverlap() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        setNode(triangleVehicle, 0, 5.0f, 0.0f, 0.0f, false);
        setNode(triangleVehicle, 1, 5.0f, 1.0f, 0.0f, false);
        setNode(triangleVehicle, 2, 5.0f, 0.0f, 1.0f, false);
        addTriangle(triangleVehicle, 0, 0, 1, 2);

        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        setNode(nodeVehicle, 0, 0.0f, 0.25f, 0.25f, true);
        nodeVehicle.nodes.velX[0] = 5.0f;

        triangleVehicle.collisionChunks.rebuild();
        nodeVehicle.collisionChunks.rebuild();
        triangleVehicle.collisionChunks.refit(1.0);
        nodeVehicle.collisionChunks.refit(1.0);
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        CollisionPipeline pipeline = new CollisionPipeline(new VoxelSnapshot(), new DynamicAxisSweep(), manager);

        pipeline.generateChunkCollisionCandidates(triangleVehicle, List.of(nodeVehicle));

        assertEquals(1, manager.contactCount.get());
    }

    @Test
    void chunkSapSkipsNodeChunksThatCannotOverlapOnItsActiveAxis() {
        SoftBodyVehicle triangleVehicle = triangleVehicleAtOrigin();
        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        for (int node = 0; node < 64; node++) {
            float x = node < 32 ? -100.0f - node : 100.0f + node;
            setNode(nodeVehicle, node, x, 0.25f, 0.25f, true);
        }

        refit(triangleVehicle, nodeVehicle);
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        CollisionPipeline pipeline = new CollisionPipeline(new VoxelSnapshot(), new DynamicAxisSweep(), manager);

        pipeline.generateChunkCollisionCandidates(triangleVehicle, List.of(nodeVehicle));

        assertTrue(nodeVehicle.collisionChunks.nodeChunkCount() > 1);
        assertEquals(0, triangleVehicle.collisionChunkPairTests,
                "the chunk SAP should reject separated chunks before their 3D AABB tests");
        assertEquals(0, manager.contactCount.get());
    }

    @Test
    void localNodeSapScansOnlyTheOneDimensionalCandidates() {
        SoftBodyVehicle triangleVehicle = triangleVehicleAtOrigin();
        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        for (int node = 0; node < 16; node++) {
            float x = node < 7 ? -100.0f - node : node == 7 ? 0.0f : 100.0f + node;
            setNode(nodeVehicle, node, x, 0.25f, 0.25f, true);
        }

        refit(triangleVehicle, nodeVehicle);
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        CollisionPipeline pipeline = new CollisionPipeline(new VoxelSnapshot(), new DynamicAxisSweep(), manager);

        pipeline.generateChunkCollisionCandidates(triangleVehicle, List.of(nodeVehicle));

        assertEquals(1, nodeVehicle.collisionChunks.nodeChunkCount());
        assertEquals(1, triangleVehicle.collisionFinePairTests,
                "the local SAP should avoid 3D tests for the other fifteen nodes");
        assertEquals(1, manager.contactCount.get());
        assertEquals(7, manager.contactNodeId[0]);
    }

    @Test
    void localNodeSapChoosesTheNarrowestAxisForEachMeshletChunkPair() {
        SoftBodyVehicle triangleVehicle = new SoftBodyVehicle(null);
        setNode(triangleVehicle, 0, -500.0f, 0.0f, 0.0f, false);
        setNode(triangleVehicle, 1, 500.0f, 0.0f, 0.0f, false);
        setNode(triangleVehicle, 2, 0.0f, 1.0f, 0.0f, false);
        addTriangle(triangleVehicle, 0, 0, 1, 2);

        SoftBodyVehicle nodeVehicle = new SoftBodyVehicle(null);
        for (int node = 0; node < 16; node++) {
            float y = node == 7 ? 0.25f : 100.0f + node;
            setNode(nodeVehicle, node, node * 30.0f, y, 0.0f, true);
        }

        refit(triangleVehicle, nodeVehicle);
        SoftBodyCollisionManager manager = new SoftBodyCollisionManager();
        CollisionPipeline pipeline = new CollisionPipeline(new VoxelSnapshot(), new DynamicAxisSweep(), manager);

        pipeline.generateChunkCollisionCandidates(triangleVehicle, List.of(nodeVehicle));

        assertEquals(1, nodeVehicle.collisionChunks.nodeChunkCount());
        assertEquals(1, triangleVehicle.collisionFinePairTests,
                "the pair should choose Y instead of scanning every node on the chunk's longest X axis");
        assertEquals(1, manager.contactCount.get());
        assertEquals(7, manager.contactNodeId[0]);
    }

    private static SoftBodyVehicle triangleVehicleAtOrigin() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        setNode(vehicle, 0, 0.0f, 0.0f, 0.0f, false);
        setNode(vehicle, 1, 0.0f, 1.0f, 0.0f, false);
        setNode(vehicle, 2, 0.0f, 0.0f, 1.0f, false);
        addTriangle(vehicle, 0, 0, 1, 2);
        return vehicle;
    }

    private static void refit(SoftBodyVehicle... vehicles) {
        for (SoftBodyVehicle vehicle : vehicles) {
            vehicle.collisionChunks.rebuild();
            vehicle.collisionChunks.refit(0.0);
        }
    }

    private static void setNode(SoftBodyVehicle vehicle, int node, float x, float y, float z, boolean collision) {
        vehicle.nodes.count = Math.max(vehicle.nodes.count, node + 1);
        vehicle.nodes.baseX[node] = vehicle.nodes.posX[node] = vehicle.nodes.prevPosX[node] = x;
        vehicle.nodes.baseY[node] = vehicle.nodes.posY[node] = vehicle.nodes.prevPosY[node] = y;
        vehicle.nodes.baseZ[node] = vehicle.nodes.posZ[node] = vehicle.nodes.prevPosZ[node] = z;
        vehicle.nodes.collision[node] = collision;
    }

    private static void addTriangle(SoftBodyVehicle vehicle, int triangle, int a, int b, int c) {
        vehicle.triangles.count = Math.max(vehicle.triangles.count, triangle + 1);
        vehicle.triangles.node1[triangle] = a;
        vehicle.triangles.node2[triangle] = b;
        vehicle.triangles.node3[triangle] = c;
        vehicle.triangles.partId[triangle] = -1;
        vehicle.triangles.collision[triangle] = true;
    }
}
