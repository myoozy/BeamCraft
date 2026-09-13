package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.physics.PhysicsSpecs;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.VehicleCameraData;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VehicleCameraControllerTest {
    @Test
    void derivesMinecraftYawFromDeformedRefBackAxis() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("ref", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(node("back", 0.0f, 0.0f, -1.0f));
        vehicle.cameras.setRefNodes(new VehicleCameraData.RefNodes(0, 1, 0, 0));

        assertEquals(0.0f, VehicleCameraController.resolveVehicleYaw(vehicle, 0.0f), 0.0001f);

        vehicle.nodes.posX[1] = 1.0f;
        vehicle.nodes.posZ[1] = 0.0f;
        assertEquals(90.0f, VehicleCameraController.resolveVehicleYaw(vehicle, 0.0f), 0.0001f);
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x, float y, float z) {
        return new PhysicsSpecs.NodeSpec(
                name, x, y, z, 1.0f, 0.5f, -1.0f,
                0, false, false, List.of());
    }
}
