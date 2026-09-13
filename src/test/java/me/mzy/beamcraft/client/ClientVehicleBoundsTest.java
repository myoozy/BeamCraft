package me.mzy.beamcraft.client;

import me.mzy.beamcraft.client.physics.NodeContainer;
import net.minecraft.util.math.Box;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
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

        Box bounds = ClientVehicleManager.computeRobustLocalBounds(nodes);

        assertEquals(-3.0, bounds.minX, 1.0e-6);
        assertEquals(3.0, bounds.maxX, 1.0e-6);
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

        Box bounds = ClientVehicleManager.computeRobustLocalBounds(nodes);

        assertEquals(97.0, bounds.minX, 1.0e-6);
        assertEquals(103.0, bounds.maxX, 1.0e-6);
    }

    @Test
    void acceptanceRadiusScalesWithOriginalVehicleSize() {
        NodeContainer nodes = nodes(
                new float[]{-10, -5, 0, 5, 10},
                new float[]{0, 0, 0, 0, 0},
                new float[]{0, 0, 0, 0, 0}
        );

        Box bounds = ClientVehicleManager.computeRobustLocalBounds(nodes);

        assertEquals(-11.0, bounds.minX, 1.0e-6);
        assertEquals(11.0, bounds.maxX, 1.0e-6);
    }

    private static NodeContainer nodes(float[] x, float[] y, float[] z) {
        NodeContainer nodes = new NodeContainer();
        nodes.count = x.length;
        for (int i = 0; i < x.length; i++) {
            nodes.renderSnapCurrX[i] = x[i];
            nodes.renderSnapCurrY[i] = y[i];
            nodes.renderSnapCurrZ[i] = z[i];
            nodes.baseX[i] = x[i];
            nodes.baseY[i] = y[i];
            nodes.baseZ[i] = z[i];
            nodes.mass[i] = 1.0f;
        }
        return nodes;
    }
}
