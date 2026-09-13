package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

class NodeContainerMedianTest {

    @Test
    void detachedHeavyOutlierDoesNotMoveRecenteringOrigin() {
        NodeContainer nodes = new NodeContainer();
        nodes.count = 5;
        nodes.posX[0] = 0;
        nodes.posX[1] = 1;
        nodes.posX[2] = 2;
        nodes.posX[3] = 3;
        nodes.posX[4] = 10_000;
        nodes.posY[0] = -2;
        nodes.posY[1] = -1;
        nodes.posY[2] = 0;
        nodes.posY[3] = 1;
        nodes.posY[4] = 5_000;
        nodes.posZ[0] = -4;
        nodes.posZ[1] = -2;
        nodes.posZ[2] = 0;
        nodes.posZ[3] = 2;
        nodes.posZ[4] = 8_000;
        for (int i = 0; i < nodes.count; i++) {
            nodes.mass[i] = 1.0f;
        }
        nodes.mass[4] = 100_000.0f;

        float[] median = new float[3];
        nodes.getMedianPosition(median);

        assertArrayEquals(new float[]{2.0f, 0.0f, 0.0f}, median, 0.0f);
    }

    @Test
    void evenNodeCountUsesMidpointOfCentralCoordinates() {
        NodeContainer nodes = new NodeContainer();
        nodes.count = 4;
        nodes.posX[0] = -10;
        nodes.posX[1] = 2;
        nodes.posX[2] = 4;
        nodes.posX[3] = 100;
        nodes.posY[0] = nodes.posY[1] = nodes.posY[2] = nodes.posY[3] = 6;
        nodes.posZ[0] = nodes.posZ[1] = nodes.posZ[2] = nodes.posZ[3] = -3;

        float[] median = new float[3];
        nodes.getMedianPosition(median);

        assertArrayEquals(new float[]{3.0f, 6.0f, -3.0f}, median, 0.0f);
    }
}
