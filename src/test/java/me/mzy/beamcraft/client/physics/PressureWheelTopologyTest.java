package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.junit.jupiter.api.Test;

import java.util.HashMap;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** Regression coverage for the staggered-ring topology in BeamNG's addPressureWheel. */
class PressureWheelTopologyTest {

    @Test
    void pressureSurfaceDiagonalsFollowTheirSupportingBeams() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        addNode(vehicle.nodes, 0, "axleOuter", -0.5f, 0.0f, 0.0f);
        addNode(vehicle.nodes, 1, "axleInner", 0.5f, 0.0f, 0.0f);

        JsonArray wheels = new JsonArray();
        JsonArray header = new JsonArray();
        for (String column : new String[]{"name", "hubGroup", "group", "node1:", "node2:",
                "nodeS", "nodeArm:", "wheelDir"}) {
            header.add(column);
        }
        wheels.add(header);

        JsonArray wheel = new JsonArray();
        wheel.add("W");
        wheel.add("hub_W");
        wheel.add("tire_W");
        wheel.add("axleOuter");
        wheel.add("axleInner");
        wheel.add(9999);
        wheel.add(9999);
        wheel.add(1);
        wheels.add(wheel);

        JsonObject config = new JsonObject();
        config.addProperty("numRays", 4);
        config.addProperty("hubRadius", 0.3);
        config.addProperty("radius", 0.5);
        config.addProperty("hubWidth", 0.3);
        config.addProperty("tireWidth", 0.4);
        config.addProperty("pressurePSI", 30.0);

        JBeamPressureWheelsParser.parsePressureWheels(wheels, vehicle,
                new JBeamAssembler.PartEntry(null, 0, "test",
                        new JBeamAssembler.TransformContext(), new HashMap<>()),
                config);

        WheelContainer generated = vehicle.wheels;
        int base = 0;
        int hIn = generated.hubInnerNodes[base];
        int hInNext = generated.hubInnerNodes[base + 1];
        int hOut = generated.hubOuterNodes[base];
        int hOutNext = generated.hubOuterNodes[base + 1];
        int tIn = generated.tireInnerNodes[base];
        int tInNext = generated.tireInnerNodes[base + 1];
        int firstTriangle = generated.tireTriangleIdxStart[0];

        // BeamNG wheels.lua: outside pressure tris use tIn_i -> hIn_{i+1}.
        assertTriangle(vehicle.triangles, firstTriangle, tIn, hIn, hInNext);
        assertTriangle(vehicle.triangles, firstTriangle + 1, tIn, hInNext, tInNext);

        // BeamNG wheels.lua: hub pressure tris use hOut_i -> hIn_{i+1}, the same
        // diagonal generated for the hub-tread beam family.
        assertTriangle(vehicle.triangles, firstTriangle + 6, hOut, hInNext, hIn);
        assertTriangle(vehicle.triangles, firstTriangle + 7, hOut, hOutNext, hInNext);
    }

    private static void assertTriangle(TriangleContainer triangles, int index,
                                       int node1, int node2, int node3) {
        assertEquals(node1, triangles.node1[index], "triangle " + index + " node1");
        assertEquals(node2, triangles.node2[index], "triangle " + index + " node2");
        assertEquals(node3, triangles.node3[index], "triangle " + index + " node3");
    }

    private static void addNode(NodeContainer nodes, int index, String name,
                                float x, float y, float z) {
        nodes.names[index] = name;
        nodes.nameToIndex.put(name, index);
        nodes.posX[index] = x;
        nodes.posY[index] = y;
        nodes.posZ[index] = z;
        nodes.baseX[index] = x;
        nodes.baseY[index] = y;
        nodes.baseZ[index] = z;
        nodes.mass[index] = 2.0f;
        nodes.count = Math.max(nodes.count, index + 1);
    }
}
