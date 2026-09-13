package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.junit.jupiter.api.Test;

import java.util.HashMap;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * The pressure-wheel parser must resolve BeamNG's drivetrain counter-torque node fields
 * (torqueCoupling/torqueArm/torqueArm2, usually inline in the wheel row and written with a
 * trailing colon) and store them on the wheel. NodeArm from the header becomes the braking
 * lever; a wheel without any of those fields keeps every reaction node at -1.
 */
class JBeamPressureWheelsReactionParseTest {

    @Test
    void parsesAndStoresPerWheelReactionNodes() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        addNode(nodes, 0, "axleO_RR", -1.0f, 0.0f, 0.0f);
        addNode(nodes, 1, "axleI_RR", 0.0f, 0.0f, 0.0f);
        addNode(nodes, 2, "axleO_FR", 1.0f, 0.0f, 0.0f);
        addNode(nodes, 3, "axleI_FR", 2.0f, 0.0f, 0.0f);
        addNode(nodes, 4, "rdiff", 0.0f, 0.0f, 0.0f);
        addNode(nodes, 5, "rx2r", -0.3f, 0.0f, 0.2f);
        addNode(nodes, 6, "rx4r", -0.3f, 0.1f, 0.4f);
        addNode(nodes, 7, "rh6r", -0.6f, 0.1f, 0.3f);

        JsonArray pressureWheels = new JsonArray();
        pressureWheels.add(headerRow());

        // Driven rear wheel: BeamNG-style inline object literal with colon-suffixed keys.
        JsonArray rr = new JsonArray();
        rr.add("RR");
        rr.add("hub_RR");
        rr.add("grp_RR");
        rr.add("axleO_RR");
        rr.add("axleI_RR");
        rr.add(9999);      // no nodeS
        rr.add("rh6r");    // nodeArm (braking lever)
        rr.add(1);         // wheelDir
        JsonObject rrMods = new JsonObject();
        rrMods.addProperty("torqueCoupling:", "rdiff");
        rrMods.addProperty("torqueArm:", "rx2r");
        rrMods.addProperty("torqueArm2:", "rx4r");
        rr.add(rrMods);
        pressureWheels.add(rr);

        // Undriven front wheel: no coupling and no braking lever.
        JsonArray fr = new JsonArray();
        fr.add("FR");
        fr.add("hub_FR");
        fr.add("grp_FR");
        fr.add("axleO_FR");
        fr.add("axleI_FR");
        fr.add(9999);      // no nodeS
        fr.add(9999);      // no nodeArm
        fr.add(1);         // wheelDir
        pressureWheels.add(fr);

        JsonObject blackboard = new JsonObject();
        blackboard.addProperty("hasTire", false);
        blackboard.addProperty("numRays", 2);

        JBeamPressureWheelsParser.parsePressureWheels(pressureWheels, vehicle,
                new JBeamAssembler.PartEntry(null, 0, "test",
                        new JBeamAssembler.TransformContext(), new HashMap<>()),
                blackboard);

        WheelContainer wheels = vehicle.wheels;
        assertEquals(2, wheels.count);

        int rrIndex = wheels.nameToIndex.get("RR");
        assertEquals(nodes.nameToIndex.get("rdiff"), wheels.torqueCouplingNode[rrIndex]);
        assertEquals(nodes.nameToIndex.get("rx2r"), wheels.torqueArmNode[rrIndex]);
        assertEquals(nodes.nameToIndex.get("rx4r"), wheels.torqueArm2Node[rrIndex]);
        assertEquals(nodes.nameToIndex.get("rh6r"), wheels.nodeArmNode[rrIndex],
                "nodeArm header column must become the braking lever");
        assertEquals(-1, wheels.nodeCouplingNode[rrIndex],
                "nodeCoupling defaults to the inner axle node, so stays undefined");

        int frIndex = wheels.nameToIndex.get("FR");
        assertEquals(-1, wheels.torqueCouplingNode[frIndex],
                "undriven wheel must not inherit the previous row's torque coupling");
        assertEquals(-1, wheels.torqueArmNode[frIndex]);
        assertEquals(-1, wheels.torqueArm2Node[frIndex]);
        assertEquals(-1, wheels.nodeArmNode[frIndex]);
        assertEquals(-1, wheels.nodeCouplingNode[frIndex]);
    }

    @Test
    void unresolvedReactionNodeNamesStayInert() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        addNode(nodes, 0, "axleO", -1.0f, 0.0f, 0.0f);
        addNode(nodes, 1, "axleI", 0.0f, 0.0f, 0.0f);

        JsonArray pressureWheels = new JsonArray();
        pressureWheels.add(headerRow());

        JsonArray wheel = new JsonArray();
        wheel.add("W");
        wheel.add("hub_W");
        wheel.add("grp_W");
        wheel.add("axleO");
        wheel.add("axleI");
        wheel.add(9999);
        wheel.add("rh6z"); // nodeArm references a node that does not exist
        wheel.add(1);
        JsonObject mods = new JsonObject();
        mods.addProperty("torqueCoupling:", "missingNode");
        mods.addProperty("torqueArm:", "axleI");
        mods.addProperty("torqueArm2:", "axleO");
        wheel.add(mods);
        pressureWheels.add(wheel);

        JsonObject blackboard = new JsonObject();
        blackboard.addProperty("hasTire", false);
        blackboard.addProperty("numRays", 2);

        JBeamPressureWheelsParser.parsePressureWheels(pressureWheels, vehicle,
                new JBeamAssembler.PartEntry(null, 0, "test",
                        new JBeamAssembler.TransformContext(), new HashMap<>()),
                blackboard);

        WheelContainer wheels = vehicle.wheels;
        int index = wheels.nameToIndex.get("W");
        assertEquals(-1, wheels.torqueCouplingNode[index],
                "an unresolved torqueCoupling disables the drivetrain reaction");
        assertEquals(nodes.nameToIndex.get("axleI"), wheels.torqueArmNode[index]);
        assertEquals(nodes.nameToIndex.get("axleO"), wheels.torqueArm2Node[index]);
        assertEquals(-1, wheels.nodeArmNode[index]);
    }

    private static JsonArray headerRow() {
        JsonArray header = new JsonArray();
        for (String column : new String[]{"name", "hubGroup", "group", "node1:", "node2:",
                "nodeS", "nodeArm:", "wheelDir"}) {
            header.add(column);
        }
        return header;
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
