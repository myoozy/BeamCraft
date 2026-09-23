package me.mzy.beamcraft.client;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.VehicleCameraData;
import net.minecraft.util.math.Box;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ClientVehicleBoundsTest {

    @Test
    void detachedOutlierDoesNotExpandEntityBounds() {
        NodeContainer nodes = nodes(
                new float[]{-2, -2, -1, -1, 1, 1, 2, 2, 1_000},
                new float[]{-1, 1, -1, 1, -1, 1, -1, 1, 0},
                new float[]{-1, -1, 1, 1, -1, -1, 1, 1, 0}
        );
        // The last node started on the body before becoming detached.
        nodes.baseX[8] = 0;

        Box bounds = ClientVehicleManager.computeCappedRenderLocalBounds(nodes, 6.0);

        assertEquals(-2.0, bounds.minX, 1.0e-6);
        assertEquals(4.0, bounds.maxX, 1.0e-6);
        assertTrue(bounds.maxX < 100.0);
    }

    @Test
    void boundsFollowTheTranslatedMainBody() {
        NodeContainer nodes = nodes(
                new float[]{98, 99, 100, 101, 102, -500},
                new float[]{0, 0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0, 0}
        );
        for (int i = 0; i < nodes.count; i++) {
            nodes.baseX[i] = i - 2.5f;
        }

        Box bounds = ClientVehicleManager.computeCappedRenderLocalBounds(nodes, 6.0);

        assertEquals(96.5, bounds.minX, 1.0e-6);
        assertEquals(102.5, bounds.maxX, 1.0e-6);
    }

    @Test
    void renderCapScalesWithOriginalVehicleSize() {
        NodeContainer nodes = nodes(
                new float[]{-10, -5, 0, 5, 10},
                new float[]{0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0}
        );

        ClientVehicleManager.BoundsProfile profile = ClientVehicleManager.computeBoundsProfile(nodes);
        Box bounds = ClientVehicleManager.computeCappedRenderLocalBounds(nodes, profile.renderMaxSpan());

        assertEquals(-10.0, bounds.minX, 1.0e-6);
        assertEquals(10.0, bounds.maxX, 1.0e-6);
        assertEquals(22.0, profile.renderMaxSpan(), 1.0e-6);
    }

    @Test
    void interactionCubeUsesDriverLongitudinalAndMedianLateralPosition() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        setNodes(nodes,
                new float[]{-0.8f, 0.0f, 0.8f, 0.0f},
                new float[]{1.2f, 0.0f, 0.0f, 0.0f},
                new float[]{-1.0f, 0.0f, 0.0f, 1.0f});
        vehicle.cameras.addInternal("driver", 0, 65.0f);
        vehicle.cameras.setRefNodes(new VehicleCameraData.RefNodes(1, 3, 2, 0));
        vehicle.interactionBoundsSide = 2.0;

        Box bounds = ClientVehicleManager.computeInteractionLocalBounds(vehicle);

        assertEquals(-1.0, bounds.minX, 1.0e-6);
        assertEquals(1.0, bounds.maxX, 1.0e-6);
        assertEquals(0.2, bounds.minY, 1.0e-6);
        assertEquals(-2.0, bounds.minZ, 1.0e-6);
        assertEquals(0.0, bounds.maxZ, 1.0e-6);
    }

    @Test
    void interactionCubeFallsBackToNodeMedianWithoutDriver() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        setNodes(vehicle.nodes,
                new float[]{8.0f, 10.0f, 12.0f},
                new float[]{1.0f, 2.0f, 3.0f},
                new float[]{-2.0f, 0.0f, 2.0f});
        vehicle.interactionBoundsSide = 2.0;

        Box bounds = ClientVehicleManager.computeInteractionLocalBounds(vehicle);

        assertEquals(9.0, bounds.minX, 1.0e-6);
        assertEquals(11.0, bounds.maxX, 1.0e-6);
        assertEquals(1.0, bounds.minY, 1.0e-6);
        assertEquals(3.0, bounds.maxY, 1.0e-6);
    }

    @Test
    void riderAnchorUsesOnlyAnExplicitDriverCamera() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        setNodes(vehicle.nodes,
                new float[]{2.0f, 6.0f, 10.0f},
                new float[]{1.0f, 3.0f, 5.0f},
                new float[]{-4.0f, 0.0f, 4.0f});
        vehicle.cameras.addInternal("hood", 0, 65.0f);
        float[] anchor = new float[3];

        assertFalse(vehicle.resolveRiderAnchor(anchor));
        assertEquals(6.0, anchor[0], 1.0e-6);
        assertEquals(3.0, anchor[1], 1.0e-6);
        assertEquals(0.0, anchor[2], 1.0e-6);

        vehicle.cameras.addInternal("driver", 2, 65.0f);
        assertTrue(vehicle.resolveRiderAnchor(anchor));
        assertEquals(10.0, anchor[0], 1.0e-6);
        assertEquals(5.0, anchor[1], 1.0e-6);
        assertEquals(4.0, anchor[2], 1.0e-6);
    }

    private static NodeContainer nodes(float[] x, float[] y, float[] z) {
        NodeContainer nodes = new NodeContainer();
        setNodes(nodes, x, y, z);
        return nodes;
    }

    private static void setNodes(NodeContainer nodes, float[] x, float[] y, float[] z) {
        nodes.count = x.length;
        for (int i = 0; i < x.length; i++) {
            nodes.renderSnapCurrX[i] = x[i];
            nodes.renderSnapCurrY[i] = y[i];
            nodes.renderSnapCurrZ[i] = z[i];
            nodes.posX[i] = x[i];
            nodes.posY[i] = y[i];
            nodes.posZ[i] = z[i];
            nodes.baseX[i] = x[i];
            nodes.baseY[i] = y[i];
            nodes.baseZ[i] = z[i];
            nodes.mass[i] = 1.0f;
        }
    }
}
