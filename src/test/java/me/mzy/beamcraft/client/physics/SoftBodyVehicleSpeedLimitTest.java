package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SoftBodyVehicleSpeedLimitTest {
    @Test
    void supersonicNodeIsHardClampedWithoutChangingDirection() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.nodes.count = 1;
        vehicle.nodes.mass[0] = 1.0f;
        vehicle.nodes.velX[0] = 300.0f;
        vehicle.nodes.velY[0] = 400.0f;

        vehicle.solveInternalForces(1.0e-6f, 1.0f);

        float x = vehicle.nodes.velX[0];
        float y = vehicle.nodes.velY[0];
        float z = vehicle.nodes.velZ[0];
        float speed = (float) Math.sqrt(x * x + y * y + z * z);
        assertTrue(speed <= SoftBodyVehicle.MAX_NODE_SPEED + 1.0e-3f);
        assertEquals(SoftBodyVehicle.MAX_NODE_SPEED, speed, 1.0e-3f);
        assertEquals(0.75f, x / y, 1.0e-4f,
                "the limiter must scale the velocity vector rather than change its direction");
    }
}
