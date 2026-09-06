package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.WheelContainer;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorsionReactorSpec;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PowertrainTorqueReactionTest {
    private static final float DT = 0.0005f;

    @Test
    void torsionReactionUsesCurrentForwardGearRatio() {
        SoftBodyVehicle vehicle = reactionRig(false, true);
        selectGear(vehicle, 2); // ratio 1, while the compile-time first ratio is 3
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(0.0f, 0.0f);

        vehicle.powertrain.solve(DT);

        float clutchTorque = vehicle.powertrain.debugClutchTorque();
        assertTrue(clutchTorque > 0.0f);
        assertEquals(clutchTorque, reactionTorqueX(vehicle.nodes), 1.0e-3f,
                "reactor torque must use the active 1:1 gear, not the compiled first gear");
    }

    @Test
    void reverseGearFlipsTorsionReaction() {
        SoftBodyVehicle vehicle = reactionRig(false, true);
        selectGear(vehicle, 3); // ratio -2
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(0.0f, 0.0f);

        vehicle.powertrain.solve(DT);

        float clutchTorque = vehicle.powertrain.debugClutchTorque();
        assertTrue(clutchTorque > 0.0f);
        assertEquals(-2.0f * clutchTorque, reactionTorqueX(vehicle.nodes), 1.0e-3f,
                "negative gearing must reverse the downstream body reaction");
    }

    @Test
    void neutralDisconnectsClutchAndTorsionReaction() {
        SoftBodyVehicle vehicle = reactionRig(false, true);
        selectGear(vehicle, 0); // neutral
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(0.0f, 0.0f);

        vehicle.powertrain.solve(DT);

        assertEquals(0.0f, vehicle.powertrain.debugClutchTorque(), 1.0e-6f);
        assertEquals(0.0f, reactionTorqueX(vehicle.nodes), 1.0e-6f,
                "neutral must disconnect every downstream drivetrain reaction");
    }

    @Test
    void neutralStillAppliesBeamNgStyleCrankInertiaReaction() {
        SoftBodyVehicle vehicle = reactionRig(true, false);
        selectGear(vehicle, 0); // neutral
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(1.0f, 0.0f);

        vehicle.powertrain.solve(DT);

        assertEquals(0.0f, vehicle.powertrain.debugClutchTorque(), 1.0e-6f);
        assertTrue(reactionTorqueX(vehicle.nodes) > 0.0f,
                "neutral disconnects the driveline, but not the engine's own crank inertia reaction");
    }

    private static SoftBodyVehicle reactionRig(boolean engineReaction, boolean torsionReaction) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        addNode(nodes, 0, "r1", 0.0f, 0.0f, 0.0f, 1.0f);
        addNode(nodes, 1, "r2", 1.0f, 0.0f, 0.0f, 1.0f);
        addNode(nodes, 2, "r3", 0.0f, 1.0f, 1.0f, 1.0f);
        addNode(nodes, 3, "axle1", 0.0f, 0.0f, 0.0f, 1.0f);
        addNode(nodes, 4, "axle2", 1.0f, 0.0f, 0.0f, 1.0f);
        addNode(nodes, 5, "hub1", 0.5f, 0.0f, 0.35f, 2.5f);
        addNode(nodes, 6, "hub2", 0.5f, 0.0f, -0.35f, 2.5f);
        addNode(nodes, 7, "hub3", 0.5f, 0.35f, 0.0f, 2.5f);
        addNode(nodes, 8, "hub4", 0.5f, -0.35f, 0.0f, 2.5f);

        vehicle.wheels.count = 1;
        vehicle.wheels.nameToIndex.put("W", 0);
        vehicle.wheels.node1[0] = 3;
        vehicle.wheels.node2[0] = 4;
        vehicle.wheels.wheelDir[0] = 1;
        vehicle.wheels.numRays[0] = 2;
        vehicle.wheels.tireRadius[0] = 0.0f;
        vehicle.wheels.hubInnerNodes[0] = 5;
        vehicle.wheels.hubOuterNodes[0] = 6;
        vehicle.wheels.hubInnerNodes[1] = 7;
        vehicle.wheels.hubOuterNodes[1] = 8;

        List<String> reactionNodes = List.of("r1", "r2", "r3");
        List<DeviceSpec> specs = new ArrayList<>();
        specs.add(new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                1.0, 0, 6000, 0, 0, 0,
                List.of(new TorquePoint(0, 100), new TorquePoint(6000, 100)),
                engineReaction ? reactionNodes : List.of(), List.of()));
        specs.add(new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                1000, 1000, 1, 0.15, 0.125, 1, List.of()));
        specs.add(new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                List.of(0.0, 3.0, 1.0, -2.0), false, 0, 0, 0, List.of()));
        String shaftInput = "gearbox";
        if (torsionReaction) {
            specs.add(new TorsionReactorSpec("torsionReactor", "reactor", "gearbox", 1,
                    1, "", 0, 0, 0, reactionNodes, List.of(), List.of()));
            shaftInput = "reactor";
        }
        specs.add(new ShaftSpec("shaft", "wheelShaft", shaftInput, 1,
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
        nodes.mass[index] = mass;
        nodes.count = Math.max(nodes.count, index + 1);
    }

    private static void selectGear(SoftBodyVehicle vehicle, int index) {
        vehicle.powertrain.gearboxes.currentGearIndex[0] = index;
        vehicle.powertrain.gearboxes.pendingGearIndex[0] = -1;
        vehicle.powertrain.gearboxes.activeRatio[0] =
                vehicle.powertrain.gearboxes.gearRatios[vehicle.powertrain.gearboxes.gearStart[0] + index];
    }

    private static float reactionTorqueX(NodeContainer nodes) {
        float torque = 0.0f;
        for (int node = 0; node < 3; node++) {
            torque += nodes.posY[node] * nodes.forceZ[node] - nodes.posZ[node] * nodes.forceY[node];
        }
        return torque;
    }
}
