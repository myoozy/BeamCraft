package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BeamBreakCommitTest {
    private static final float DT = 1.0e-3f;

    @Test
    void breakGroupCommitDoesNotLetBeamOrderControlPeerForce() {
        SoftBodyVehicle weakFirst = vehicleWithGroupedBeams(true);
        SoftBodyVehicle strongFirst = vehicleWithGroupedBeams(false);

        weakFirst.solveInternalForces(DT, 0.0f);
        strongFirst.solveInternalForces(DT, 0.0f);

        for (String nodeName : List.of("a", "b", "c")) {
            int weakFirstNode = weakFirst.nodes.nameToIndex.get(nodeName);
            int strongFirstNode = strongFirst.nodes.nameToIndex.get(nodeName);
            assertEquals(weakFirst.nodes.velX[weakFirstNode], strongFirst.nodes.velX[strongFirstNode], 1.0e-7f,
                    "beam insertion order must not decide whether a break-group peer contributes force");
        }

        assertEquals(0.05f, weakFirst.nodes.velX[weakFirst.nodes.nameToIndex.get("a")], 1.0e-6f,
                "the intact peer contributes its force before the fracture commit");
        assertAllGroupedBeamsBroken(weakFirst);
        assertAllGroupedBeamsBroken(strongFirst);
        assertEquals(0, weakFirst.normalBeams.pendingBreakCount());
        assertEquals(0, strongFirst.normalBeams.pendingBreakCount());
    }

    private static SoftBodyVehicle vehicleWithGroupedBeams(boolean weakFirst) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0.0f));
        vehicle.addNode(node("b", 1.0f));
        vehicle.addNode(node("c", 2.0f));

        PhysicsSpecs.BeamSpec weak = beam("b", "c", 1.0f);
        PhysicsSpecs.BeamSpec strong = beam("a", "b", 1_000.0f);
        if (weakFirst) {
            vehicle.addBeam(weak);
            vehicle.addBeam(strong);
        } else {
            vehicle.addBeam(strong);
            vehicle.addBeam(weak);
        }
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x) {
        return new PhysicsSpecs.NodeSpec(name, x, 0.0f, 0.0f,
                1.0f, 1.0f, 1.0f, 0, false, false, List.of());
    }

    private static PhysicsSpecs.BeamSpec beam(String node1, String node2, float strength) {
        return new PhysicsSpecs.BeamSpec(
                BeamContainer.BEAM_NORMAL, node1, node2, null,
                List.of(), Float.POSITIVE_INFINITY,
                List.of("shared"), 0, false,
                100.0f, 0.0f, Float.MAX_VALUE, strength,
                0.5f, 0.0f, 0.0f,
                1.0f, 1.0f, -1.0f, -1.0f, 1.0f,
                0.0f, 0.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
                100.0f, 0.0f, 0.0f, Float.MAX_VALUE);
    }

    private static void assertAllGroupedBeamsBroken(SoftBodyVehicle vehicle) {
        assertEquals(2, vehicle.normalBeams.count);
        assertTrue(vehicle.normalBeams.broken[0]);
        assertTrue(vehicle.normalBeams.broken[1]);
    }
}
