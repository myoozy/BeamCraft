package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BeamGraphTest {

    @Test
    void distinguishesCohesiveEdgesFromDetachableAndSupportEdges() {
        SoftBodyVehicle vehicle = vehicleWithNodes(5);
        addEdge(vehicle.normalBeams, 0, 1, null);
        addEdge(vehicle.normalBeams, 1, 2, List.of("bumper"));
        addEdge(vehicle.supportBeams, 2, 3, null);
        addEdge(vehicle.normalBeams, 0, 4, null);
        addEdge(vehicle.normalBeams, 4, 2, null);

        BeamGraph graph = BeamGraph.from(vehicle);

        assertTrue(graph.directlyConnected(0, 1));
        assertTrue(graph.cohesivelyConnected(0, 1));
        assertTrue(graph.connectedOnlyAcrossBreakGroup(1, 2));
        assertTrue(graph.directlyConnected(2, 3));
        assertFalse(graph.cohesivelyConnected(2, 3));
        assertEquals(2, graph.hopDistance(0, 2, 2, true));
        assertEquals(-1, graph.hopDistance(0, 3, 3, true));
    }

    private static SoftBodyVehicle vehicleWithNodes(int count) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.nodes.count = count;
        return vehicle;
    }

    private static void addEdge(BeamContainer container, int first, int second, List<String> breakGroups) {
        int beam = container.count++;
        container.node1[beam] = first;
        container.node2[beam] = second;
        container.assignedBreakGroups[beam] = breakGroups;
    }
}
