package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.NodeContainer;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class FlexbodyBindingUtilTest {

    @Test
    void choosesAWellConditionedPairInsteadOfTheTwoNearestCollinearNodes() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(0.30, 0, 0),
                point(0.35, 0.03, 0),
                point(0, 0.50, 0));
        FlexbodyContainer flex = flexForOneVertex();

        assertTrue(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.10, 0.05, 0, 0, 0, 1, List.of(0, 1, 2, 3)));

        assertTrue(selectedAcuteAngleDegrees(flex, nodes) >= FlexbodyBindingUtil.PREFERRED_BASIS_ANGLE_DEGREES);
        assertRestPositionReconstructs(flex, nodes, 0.10, 0.05, 0);
    }

    @Test
    void prefersLocatorCoordinatesInsideBeamNgDebugBounds() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(0.10, 0, 0),
                point(0, 0.10, 0),
                point(1, 0, 0),
                point(0, 1, 0));
        FlexbodyContainer flex = flexForOneVertex();

        assertTrue(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.50, 0.50, 0, 0, 0, 1, List.of(0, 1, 2, 3, 4)));

        assertTrue(flex.vWeightX[0] >= FlexbodyBindingUtil.PREFERRED_LOCATOR_MIN);
        assertTrue(flex.vWeightX[0] <= FlexbodyBindingUtil.PREFERRED_LOCATOR_MAX);
        assertTrue(flex.vWeightY[0] >= FlexbodyBindingUtil.PREFERRED_LOCATOR_MIN);
        assertTrue(flex.vWeightY[0] <= FlexbodyBindingUtil.PREFERRED_LOCATOR_MAX);
        assertRestPositionReconstructs(flex, nodes, 0.50, 0.50, 0);
    }

    @Test
    void rejectsACollinearNodePool() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(1, 0, 0),
                point(2, 0, 0),
                point(3, 0, 0));
        FlexbodyContainer flex = flexForOneVertex();

        assertFalse(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.25, 0, 0, 0, 1, 0, List.of(0, 1, 2, 3)));
    }

    @Test
    void usesARealFourthNodeForAThreeDimensionalLocator() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(1, 0, 0),
                point(0, 1, 0),
                point(0, 0, 1));
        FlexbodyContainer flex = flexForOneVertex();

        assertTrue(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.2, 0.3, 0.4, 0, 0, 1, List.of(0, 1, 2, 3)));

        assertFalse(flex.vUseCrossZ[0]);
        assertTrue(flex.vVzNode[0] >= 0);
        assertRestPositionReconstructs(flex, nodes, 0.2, 0.3, 0.4);
    }

    @Test
    void fallsBackToCrossNormalWhenExplicitZWouldAmplifyNodeMotion() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(1, 0, 0),
                point(0, 1, 0),
                point(1, 1, 0.6));
        FlexbodyContainer flex = flexForOneVertex();

        assertTrue(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.2, 0.2, 0.5, 0, 0, 1, List.of(0, 1, 2, 3)));

        assertTrue(flex.vUseCrossZ[0]);
        assertEquals(-1, flex.vVzNode[0]);
        assertTrue(FlexbodyBindingUtil.affineNodeGain(
                flex.vWeightX[0], flex.vWeightY[0], 0.0) <= FlexbodyBindingUtil.MAX_AFFINE_NODE_GAIN);
        assertRestPositionReconstructs(flex, nodes, 0.2, 0.2, 0.5);
    }

    @Test
    void affineGainCountsCenterNodeExtrapolation() {
        assertEquals(7.592, FlexbodyBindingUtil.affineNodeGain(-1.765, -1.531, 0.706), 1.0e-3);
    }

    @Test
    void crossNormalPositionWeightUsesMetricOffset() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(2, 0, 0),
                point(0, 3, 0));
        FlexbodyContainer flex = flexForOneVertex();

        assertTrue(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.2, 0.3, 0.6, 0, 0, 1, List.of(0, 1, 2)));

        assertTrue(flex.vUseCrossZ[0]);
        assertEquals(0.6, flex.vWeightZ[0], 1.0e-6);
        assertRestPositionReconstructs(flex, nodes, 0.2, 0.3, 0.6);
    }

    @Test
    void ordinaryBodyBindingCanDisableExplicitFourthNode() {
        NodeContainer nodes = nodes(
                point(0, 0, 0),
                point(1, 0, 0),
                point(0, 1, 0),
                point(0, 0, 1));
        FlexbodyContainer flex = flexForOneVertex();

        assertTrue(FlexbodyBindingUtil.calculateDecoupledWeights(
                flex, nodes, 0, 0.2, 0.3, 0.4, 0, 0, 1,
                false, List.of(0, 1, 2, 3)));

        assertTrue(flex.vUseCrossZ[0]);
        assertEquals(-1, flex.vVzNode[0]);
        assertRestPositionReconstructs(flex, nodes, 0.2, 0.3, 0.4);
    }

    private static FlexbodyContainer flexForOneVertex() {
        FlexbodyContainer flex = new FlexbodyContainer();
        flex.allocateSkinningBuffers(1);
        return flex;
    }

    private static NodeContainer nodes(double[]... positions) {
        NodeContainer nodes = new NodeContainer();
        nodes.count = positions.length;
        for (int i = 0; i < positions.length; i++) {
            nodes.baseX[i] = (float) positions[i][0];
            nodes.baseY[i] = (float) positions[i][1];
            nodes.baseZ[i] = (float) positions[i][2];
        }
        return nodes;
    }

    private static double[] point(double x, double y, double z) {
        return new double[]{x, y, z};
    }

    private static double selectedAcuteAngleDegrees(FlexbodyContainer flex, NodeContainer nodes) {
        int center = flex.vCenterNode[0], vx = flex.vVxNode[0], vy = flex.vVyNode[0];
        double ux = nodes.baseX[vx] - nodes.baseX[center];
        double uy = nodes.baseY[vx] - nodes.baseY[center];
        double uz = nodes.baseZ[vx] - nodes.baseZ[center];
        double vxv = nodes.baseX[vy] - nodes.baseX[center];
        double vyv = nodes.baseY[vy] - nodes.baseY[center];
        double vzv = nodes.baseZ[vy] - nodes.baseZ[center];
        double cosine = Math.abs(ux * vxv + uy * vyv + uz * vzv)
                / Math.sqrt((ux * ux + uy * uy + uz * uz) * (vxv * vxv + vyv * vyv + vzv * vzv));
        return Math.toDegrees(Math.acos(Math.min(1, cosine)));
    }

    private static void assertRestPositionReconstructs(FlexbodyContainer flex, NodeContainer nodes,
                                                       double expectedX, double expectedY, double expectedZ) {
        int center = flex.vCenterNode[0], vx = flex.vVxNode[0], vy = flex.vVyNode[0];
        double ux = nodes.baseX[vx] - nodes.baseX[center];
        double uy = nodes.baseY[vx] - nodes.baseY[center];
        double uz = nodes.baseZ[vx] - nodes.baseZ[center];
        double vxv = nodes.baseX[vy] - nodes.baseX[center];
        double vyv = nodes.baseY[vy] - nodes.baseY[center];
        double vzv = nodes.baseZ[vy] - nodes.baseZ[center];
        double nx, ny, nz;
        if (flex.vUseCrossZ[0]) {
            nx = uy * vzv - uz * vyv;
            ny = uz * vxv - ux * vzv;
            nz = ux * vyv - uy * vxv;
            double inverseNormalLength = 1.0 / Math.sqrt(nx * nx + ny * ny + nz * nz);
            nx *= inverseNormalLength;
            ny *= inverseNormalLength;
            nz *= inverseNormalLength;
        } else {
            int vzNode = flex.vVzNode[0];
            nx = nodes.baseX[vzNode] - nodes.baseX[center];
            ny = nodes.baseY[vzNode] - nodes.baseY[center];
            nz = nodes.baseZ[vzNode] - nodes.baseZ[center];
        }

        assertEquals(expectedX, nodes.baseX[center] + ux * flex.vWeightX[0]
                + vxv * flex.vWeightY[0] + nx * flex.vWeightZ[0], 1.0e-6);
        assertEquals(expectedY, nodes.baseY[center] + uy * flex.vWeightX[0]
                + vyv * flex.vWeightY[0] + ny * flex.vWeightZ[0], 1.0e-6);
        assertEquals(expectedZ, nodes.baseZ[center] + uz * flex.vWeightX[0]
                + vzv * flex.vWeightY[0] + nz * flex.vWeightZ[0], 1.0e-6);
    }
}
