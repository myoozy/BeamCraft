package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class TorqueReactionSolverTest {
    @Test
    void createsRequestedTorqueWithoutNetForce() {
        NodeContainer nodes = new NodeContainer();
        nodes.count = 4;
        float[][] positions = {{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}};
        for (int i = 0; i < positions.length; i++) {
            nodes.posX[i] = positions[i][0];
            nodes.posY[i] = positions[i][1];
            nodes.posZ[i] = positions[i][2];
            nodes.mass[i] = 1.0f;
        }

        TorqueReactionSolver.apply(nodes, new int[]{0, 1, 2, 3}, 0, 4, 3.0f, -2.0f, 7.0f);

        double fx = 0.0, fy = 0.0, fz = 0.0;
        double tx = 0.0, ty = 0.0, tz = 0.0;
        for (int i = 0; i < 4; i++) {
            fx += nodes.forceX[i]; fy += nodes.forceY[i]; fz += nodes.forceZ[i];
            tx += nodes.posY[i] * nodes.forceZ[i] - nodes.posZ[i] * nodes.forceY[i];
            ty += nodes.posZ[i] * nodes.forceX[i] - nodes.posX[i] * nodes.forceZ[i];
            tz += nodes.posX[i] * nodes.forceY[i] - nodes.posY[i] * nodes.forceX[i];
        }
        assertEquals(0.0, fx, 1e-5); assertEquals(0.0, fy, 1e-5); assertEquals(0.0, fz, 1e-5);
        assertEquals(3.0, tx, 1e-4); assertEquals(-2.0, ty, 1e-4); assertEquals(7.0, tz, 1e-4);
    }
}
