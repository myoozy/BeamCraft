package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CouplerContainerTest {

    @Test
    void attachedConstraintCancelsRelativeVelocityAndPreservesMomentum() {
        NodeContainer nodes = twoNodes(0.001f);
        nodes.velX[0] = 1.0f;
        nodes.velX[1] = -1.0f;
        CouplerContainer couplers = coupler(nodes, 10_000.0f, 0.005f, 0.2f);

        float momentumBefore = nodes.mass[0] * nodes.velX[0] + nodes.mass[1] * nodes.velX[1];
        couplers.solveVelocityConstraints(nodes, 0.0005f);
        float momentumAfter = nodes.mass[0] * nodes.velX[0] + nodes.mass[1] * nodes.velX[1];

        assertEquals(CouplerContainer.ATTACHED, couplers.state[0]);
        assertEquals(nodes.velX[0], nodes.velX[1], 1e-6f);
        assertEquals(momentumBefore, momentumAfter, 1e-6f);
    }

    @Test
    void excessiveRequiredForceBreaksBeforeApplyingImpulse() {
        NodeContainer nodes = twoNodes(0.001f);
        nodes.velX[0] = 1.0f;
        nodes.velX[1] = -1.0f;
        CouplerContainer couplers = coupler(nodes, 1_000.0f, 0.005f, 0.2f);

        couplers.solveVelocityConstraints(nodes, 0.0005f);

        assertEquals(CouplerContainer.BROKEN, couplers.state[0]);
        assertEquals(1.0f, nodes.velX[0], 0.0f);
        assertEquals(-1.0f, nodes.velX[1], 0.0f);
    }

    @Test
    void latchingUsesConfiguredClosingSpeed() {
        NodeContainer nodes = twoNodes(0.1f);
        CouplerContainer couplers = coupler(nodes, 1_000.0f, 0.005f, 0.2f);

        couplers.solveVelocityConstraints(nodes, 0.0005f);

        assertEquals(CouplerContainer.LATCHING, couplers.state[0]);
        assertEquals(-0.2f, nodes.velX[1] - nodes.velX[0], 1e-6f);
    }

    @Test
    void remainsFiniteWhileLatchingAndHoldingAtPhysicsRate() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0.0f));
        vehicle.addNode(node("b", 0.1f));
        assertTrue(vehicle.addCoupler(new PhysicsSpecs.CouplerSpec(
                "a", "b", 1_000.0f, 0.2f, 0.005f, 0.2f, null)));

        for (int step = 0; step < 4_000; step++) {
            vehicle.solveInternalForces(0.0005f, 1.0f);
        }

        float separation = Math.abs(vehicle.nodes.posX[1] - vehicle.nodes.posX[0]);
        assertEquals(CouplerContainer.ATTACHED, vehicle.couplers.state[0]);
        assertTrue(separation <= 0.0051f, "separation=" + separation);
        for (int i = 0; i < vehicle.nodes.count; i++) {
            assertTrue(Float.isFinite(vehicle.nodes.posX[i]));
            assertTrue(Float.isFinite(vehicle.nodes.posY[i]));
            assertTrue(Float.isFinite(vehicle.nodes.posZ[i]));
            assertTrue(Float.isFinite(vehicle.nodes.velX[i]));
            assertTrue(Float.isFinite(vehicle.nodes.velY[i]));
            assertTrue(Float.isFinite(vehicle.nodes.velZ[i]));
        }
    }

    private static CouplerContainer coupler(NodeContainer nodes, float strength,
                                             float lockRadius, float latchSpeed) {
        CouplerContainer result = new CouplerContainer();
        result.add(new PhysicsSpecs.CouplerSpec(
                "a", "b", strength, 0.2f, lockRadius, latchSpeed, null), 0, 1);
        return result;
    }

    private static NodeContainer twoNodes(float separation) {
        NodeContainer result = new NodeContainer();
        result.addNode(node("a", 0.0f));
        result.addNode(node("b", separation));
        return result;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x) {
        return new PhysicsSpecs.NodeSpec(
                name, x, 0.0f, 0.0f,
                1.0f, 0.5f, 0.5f,
                0, false, false, List.of());
    }
}
