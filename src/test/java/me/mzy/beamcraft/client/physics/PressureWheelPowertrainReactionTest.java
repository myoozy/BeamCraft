package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end check that the drivetrain path really closes the wheel torque on the
 * pressure-wheel coupling nodes: engaged gear produces an equal-and-opposite zero-net-force
 * reaction, neutral applies no wheel torque and therefore no reaction.
 */
class PressureWheelPowertrainReactionTest {
    private static final float DT = 0.0005f;

    @Test
    void engagedGearClosesWheelTorqueOnTheCouplingNodes() {
        SoftBodyVehicle vehicle = drivetrainRig();
        selectGear(vehicle, 1); // ratio 1.0
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(0.0f, 0.0f);

        vehicle.powertrain.solve(DT);

        assertTrue(vehicle.powertrain.debugClutchTorque() > 0.0f,
                "the engaged drivetrain must transmit torque");

        double[] hubMoment = momentAboutAxle(vehicle.nodes, 5, 8);
        double[] reactionMoment = momentAboutAxle(vehicle.nodes, 0, 2);
        assertTrue(hubMoment[0] > 1.0e-3f, "driven wheel hub must receive drive torque");
        assertEquals(hubMoment[0], -reactionMoment[0], 1e-2,
                "wheel coupling reaction must be the exact opposite of the wheel torque");
        assertEquals(0.0, reactionMoment[1], 1e-2);
        assertEquals(0.0, reactionMoment[2], 1e-2);

        double[] reactionForce = netForce(vehicle.nodes, 0, 2);
        assertEquals(0.0, reactionForce[0], 1e-3);
        assertEquals(0.0, reactionForce[1], 1e-3);
        assertEquals(0.0, reactionForce[2], 1e-3);
    }

    @Test
    void neutralDisconnectsTheWheelCounterTorque() {
        SoftBodyVehicle vehicle = drivetrainRig();
        selectGear(vehicle, 0); // neutral (ratio 0)
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(1.0f, 0.0f);

        vehicle.powertrain.solve(DT);

        assertEquals(0.0f, vehicle.powertrain.debugClutchTorque(), 1.0e-6f);
        for (int node = 0; node < 9; node++) {
            assertEquals(0.0f, vehicle.nodes.forceX[node], 1.0e-6f, "forceX node " + node);
            assertEquals(0.0f, vehicle.nodes.forceY[node], 1.0e-6f, "forceY node " + node);
            assertEquals(0.0f, vehicle.nodes.forceZ[node], 1.0e-6f, "forceZ node " + node);
        }
    }

    private static SoftBodyVehicle drivetrainRig() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        addNode(nodes, 0, "c0", 0.0f, 0.6f, 1.0f, 3.0f);
        addNode(nodes, 1, "c1", 0.0f, -0.6f, 1.0f, 3.0f);
        addNode(nodes, 2, "c2", 0.0f, 0.0f, 0.4f, 3.0f);
        addNode(nodes, 3, "axle1", 0.0f, 0.0f, 0.0f, 4.0f);
        addNode(nodes, 4, "axle2", 1.0f, 0.0f, 0.0f, 4.0f);
        addNode(nodes, 5, "hub1", 0.5f, 0.0f, 0.35f, 2.5f);
        addNode(nodes, 6, "hub2", 0.5f, 0.0f, -0.35f, 2.5f);
        addNode(nodes, 7, "hub3", 0.5f, 0.35f, 0.0f, 2.5f);
        addNode(nodes, 8, "hub4", 0.5f, -0.35f, 0.0f, 2.5f);

        WheelContainer wheels = vehicle.wheels;
        wheels.count = 1;
        wheels.nameToIndex.put("W", 0);
        wheels.node1[0] = 3;
        wheels.node2[0] = 4;
        wheels.wheelDir[0] = 1;
        wheels.numRays[0] = 2;
        wheels.tireRadius[0] = 0.0f;
        wheels.hubInnerNodes[0] = 5;
        wheels.hubOuterNodes[0] = 6;
        wheels.hubInnerNodes[1] = 7;
        wheels.hubOuterNodes[1] = 8;
        wheels.setReactionNodes(0, 0, 1, 2);

        List<DeviceSpec> specs = new ArrayList<>();
        specs.add(new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                1.0, 0, 6000, 0, 0, 0,
                List.of(new TorquePoint(0, 100), new TorquePoint(6000, 100)),
                List.of(), List.of()));
        specs.add(new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                1000, 1000, 1, 0.15, 0.125, 1, List.of()));
        specs.add(new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                List.of(0.0, 1.0), false, 0, 0, 0, List.of()));
        specs.add(new ShaftSpec("shaft", "wheelShaft", "gearbox", 1,
                1, "W", 0, 0, 0, List.of(), List.of(), List.of()));
        vehicle.powertrain.addSpecs(specs);
        vehicle.powertrain.finalizeSetup();
        return vehicle;
    }

    private static void addNode(NodeContainer nodes, int index, String name,
                                float x, float y, float z, float mass) {
        nodes.names[index] = name;
        nodes.nameToIndex.put(name, index);
        nodes.posX[index] = x;
        nodes.posY[index] = y;
        nodes.posZ[index] = z;
        nodes.baseX[index] = x;
        nodes.baseY[index] = y;
        nodes.baseZ[index] = z;
        nodes.mass[index] = mass;
        nodes.count = Math.max(nodes.count, index + 1);
    }

    private static void selectGear(SoftBodyVehicle vehicle, int index) {
        vehicle.powertrain.gearboxes.currentGearIndex[0] = index;
        vehicle.powertrain.gearboxes.pendingGearIndex[0] = -1;
        vehicle.powertrain.gearboxes.activeRatio[0] =
                vehicle.powertrain.gearboxes.gearRatios[vehicle.powertrain.gearboxes.gearStart[0] + index];
    }

    private static double[] momentAboutAxle(NodeContainer nodes, int first, int last) {
        double[] sum = new double[3];
        for (int i = first; i <= last; i++) {
            sum[0] += nodes.posY[i] * nodes.forceZ[i] - nodes.posZ[i] * nodes.forceY[i];
            sum[1] += nodes.posZ[i] * nodes.forceX[i] - nodes.posX[i] * nodes.forceZ[i];
            sum[2] += nodes.posX[i] * nodes.forceY[i] - nodes.posY[i] * nodes.forceX[i];
        }
        return sum;
    }

    private static double[] netForce(NodeContainer nodes, int first, int last) {
        double[] sum = new double[3];
        for (int i = first; i <= last; i++) {
            sum[0] += nodes.forceX[i];
            sum[1] += nodes.forceY[i];
            sum[2] += nodes.forceZ[i];
        }
        return sum;
    }
}
