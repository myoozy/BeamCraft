package me.mzy.beamcraft.client;

import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

class VehicleMemoryEstimatorTest {

    @Test
    void growsWithVehicleArraysAndEstimatedGpuVertices() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        long baseline = VehicleMemoryEstimator.estimateRetainedBytes(vehicle);

        vehicle.nodes.posX = new float[4096];
        vehicle.nodes.count = 512;
        vehicle.flexbodies.totalVertexCount = 10_000;
        long expanded = VehicleMemoryEstimator.estimateRetainedBytes(vehicle);

        assertTrue(expanded > baseline + 1_000_000L,
                "expanded physics/GPU state should materially increase the cache estimate");
    }
}
