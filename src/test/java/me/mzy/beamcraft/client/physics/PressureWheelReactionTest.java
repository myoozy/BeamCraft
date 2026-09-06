package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Mechanics of the BeamNG pressure-wheel counter-torque: the reaction of a wheel torque
 * must be equal and opposite, produce no net force, follow the forward-positive axle
 * convention on both wheelDir sides, and stay inert for absent/degenerate configuration.
 */
class PressureWheelReactionTest {
    private static final int AXLE_OUTER = 0;
    private static final int AXLE_INNER = 1;
    private static final int HUB0 = 2;
    private static final int HUB1 = 3;
    private static final int HUB2 = 4;
    private static final int HUB3 = 5;
    private static final int COUPLING = 6;
    private static final int ARM = 7;
    private static final int ARM2 = 8;

    @Test
    void driveReactionIsEqualAndOppositeWithoutNetForce() {
        SoftBodyVehicle vehicle = reactionRig(false);
        WheelContainer wheels = vehicle.wheels;
        wheels.setReactionNodes(0, COUPLING, ARM, ARM2);

        wheels.applyDriveReaction(0, 100.0f);

        double[] force = netForce(vehicle.nodes, COUPLING, ARM, ARM2);
        assertEquals(0.0, force[0], 1e-4);
        assertEquals(0.0, force[1], 1e-4);
        assertEquals(0.0, force[2], 1e-4);

        // Positive forward drive torque +100 about +x must produce -100 about +x on the
        // reaction nodes (about any reference point, since net force is zero).
        double[] moment = moment(vehicle.nodes, COUPLING, ARM, ARM2);
        assertEquals(-100.0, moment[0], 1e-2);
        assertEquals(0.0, moment[1], 1e-2);
        assertEquals(0.0, moment[2], 1e-2);
    }

    @Test
    void bothWheelDirSidesShareTheForwardPositiveReactionAxis() {
        for (boolean mirrored : new boolean[]{false, true}) {
            SoftBodyVehicle vehicle = reactionRig(mirrored);
            WheelContainer wheels = vehicle.wheels;
            wheels.setReactionNodes(0, COUPLING, ARM, ARM2);

            wheels.applyDriveReaction(0, 100.0f);

            double[] moment = moment(vehicle.nodes, COUPLING, ARM, ARM2);
            assertEquals(-100.0, moment[0], 1e-2, "forward-positive reaction on mirrored side");
            assertEquals(0.0, moment[1], 1e-2);
            assertEquals(0.0, moment[2], 1e-2);
        }
    }

    @Test
    void combinedDriveTorqueClosesTheWheelMoment() {
        SoftBodyVehicle vehicle = reactionRig(false);
        WheelContainer wheels = vehicle.wheels;
        wheels.setReactionNodes(0, COUPLING, ARM, ARM2);

        wheels.applyDriveTorqueAndReaction(0, 100.0f);

        double[] hubMoment = moment(vehicle.nodes, HUB0, HUB1, HUB2, HUB3);
        double[] reactionMoment = moment(vehicle.nodes, COUPLING, ARM, ARM2);
        assertEquals(100.0, hubMoment[0], 1e-1);
        assertEquals(-100.0, reactionMoment[0], 1e-1);
        assertEquals(0.0, hubMoment[0] + reactionMoment[0], 1e-1,
                "hub torque and reaction must cancel");
    }

    @Test
    void noDriveReactionWithoutTorqueCouplingOrArm() {
        // Wheel without any reaction nodes configured (the common, non-driven case).
        SoftBodyVehicle none = reactionRig(false);
        none.wheels.applyDriveReaction(0, 100.0f);
        assertAllForcesZero(none.nodes, 0, 8);

        // Wheel with a coupling but no torqueArm must also stay inert.
        SoftBodyVehicle armOnly = reactionRig(false);
        armOnly.wheels.setReactionNodes(0, COUPLING, -1, -1);
        armOnly.wheels.applyDriveReaction(0, 100.0f);
        assertAllForcesZero(armOnly.nodes, 0, 8);

        // Zero wheel torque (e.g. neutral upstream) implies no reaction at all.
        SoftBodyVehicle noTorque = reactionRig(false);
        noTorque.wheels.setReactionNodes(0, COUPLING, ARM, ARM2);
        noTorque.wheels.applyDriveTorqueAndReaction(0, 0.0f);
        assertAllForcesZero(noTorque.nodes, 0, 8);
    }

    @Test
    void degenerateReactionGeometryIsSafe() {
        // Coupling == arm == inner axle node: only one unique node, so no reaction forces.
        SoftBodyVehicle duplicate = reactionRig(false);
        duplicate.wheels.setReactionNodes(0, AXLE_INNER, AXLE_INNER, AXLE_INNER);
        duplicate.wheels.applyDriveReaction(0, 100.0f);
        assertAllForcesZero(duplicate.nodes, 0, 8);

        // A reaction node with zero mass cannot carry load; leave arm2 zero-mass.
        SoftBodyVehicle lightNode = reactionRig(false);
        lightNode.nodes.mass[ARM2] = 0.0f;
        lightNode.wheels.setReactionNodes(0, COUPLING, ARM, ARM2);
        lightNode.wheels.applyDriveReaction(0, 100.0f);
        double[] force = netForce(lightNode.nodes, COUPLING, ARM);
        assertEquals(0.0, force[0], 1e-4);
        assertEquals(0.0, force[1], 1e-4);
        assertEquals(0.0, force[2], 1e-4);
        double[] moment = moment(lightNode.nodes, COUPLING, ARM);
        assertEquals(-100.0, moment[0], 1e-2);

        // Degenerate axle (node1 == node2) means no axis, so nothing is applied.
        SoftBodyVehicle degenerateAxis = reactionRig(false);
        degenerateAxis.nodes.posX[AXLE_OUTER] = 0.0f; // collapse the axle
        degenerateAxis.wheels.setReactionNodes(0, COUPLING, ARM, ARM2);
        degenerateAxis.wheels.applyDriveReaction(0, 100.0f);
        assertAllForcesZero(degenerateAxis.nodes, 0, 8);
    }

    @Test
    void torqueArm2DefaultsToTheInnerAxleNode() {
        SoftBodyVehicle vehicle = reactionRig(false);
        WheelContainer wheels = vehicle.wheels;
        wheels.setReactionNodes(0, COUPLING, ARM, -1);

        wheels.applyDriveReaction(0, 100.0f);

        // Inner axle node joins the reaction set as the torqueArm2 default.
        double[] force = netForce(vehicle.nodes, COUPLING, ARM, AXLE_INNER);
        assertEquals(0.0, force[0], 1e-4);
        assertEquals(0.0, force[1], 1e-4);
        assertEquals(0.0, force[2], 1e-4);
        double[] moment = moment(vehicle.nodes, COUPLING, ARM, AXLE_INNER);
        assertEquals(-100.0, moment[0], 1e-2);
    }

    @Test
    void brakingCounterTorqueOpposesTheAppliedBrakeTorque() {
        SoftBodyVehicle vehicle = reactionRig(false);
        WheelContainer wheels = vehicle.wheels;
        // nodeArm header lever and the inner-axle nodeCoupling default.
        wheels.nodeArmNode[0] = ARM;
        wheels.nodeCouplingNode[0] = -1;
        wheels.brakeTorque[0] = 50_000.0f;
        wheels.parkingTorque[0] = 0.0f;
        wheels.brakeSpring[0] = 10.0f;
        wheels.brakeInputSplit[0] = 1.0f;
        wheels.brakeSplitCoef[0] = 1.0f;
        spinWheelForward(vehicle.nodes, 20.0f);

        assertTrue(wheels.getAngularVelocity(0) > 0.0f);
        float angularVelocityBefore = wheels.getAngularVelocity(0);

        wheels.applyServiceBrakes(1.0f, 0.005f);
        integrateForces(vehicle.nodes, 0.005f);

        assertTrue(wheels.getAngularVelocity(0) < angularVelocityBefore,
                "service brake must slow the forward-rolling wheel");

        double[] hubMoment = moment(vehicle.nodes, HUB0, HUB1, HUB2, HUB3);
        double[] reactionMoment = moment(vehicle.nodes, AXLE_INNER, ARM);
        assertEquals(0.0, hubMoment[0] + reactionMoment[0], 1e-1,
                "brake torque and its counter-torque must cancel about the axle");
        double[] reactionForce = netForce(vehicle.nodes, AXLE_INNER, ARM);
        assertEquals(0.0, reactionForce[0], 1e-3);
        assertEquals(0.0, reactionForce[1], 1e-3);
        assertEquals(0.0, reactionForce[2], 1e-3);
    }

    @Test
    void noBrakeReactionWithoutANodeArm() {
        SoftBodyVehicle vehicle = reactionRig(false);
        WheelContainer wheels = vehicle.wheels;
        wheels.nodeArmNode[0] = -1; // wheel without a brake lever
        wheels.nodeCouplingNode[0] = COUPLING;
        wheels.brakeTorque[0] = 50_000.0f;
        wheels.brakeSpring[0] = 10.0f;
        wheels.brakeInputSplit[0] = 1.0f;
        wheels.brakeSplitCoef[0] = 1.0f;
        spinWheelForward(vehicle.nodes, 20.0f);

        wheels.applyServiceBrakes(1.0f, 0.005f);

        // No explicit brake reaction: coupling must stay untouched.
        assertForcesZero(vehicle.nodes, COUPLING);
        assertForcesZero(vehicle.nodes, ARM);
    }

    /**
     * Builds a single-wheel vehicle. When {@code mirrored} is true the axle is placed on the
     * opposite side with wheelDir = -1, so the wheel reports the same forward-positive axis.
     * Hub ring nodes sit in the plane of the wheel so drive/brake torques and the reported
     * angular velocity share the forward-positive convention.
     */
    private static SoftBodyVehicle reactionRig(boolean mirrored) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        float side = mirrored ? 1.0f : -1.0f;
        addNode(nodes, AXLE_OUTER, "axleOuter", side, 0.0f, 0.0f, 4.0f);
        addNode(nodes, AXLE_INNER, "axleInner", 0.0f, 0.0f, 0.0f, 4.0f);
        float center = side * 0.5f;
        addNode(nodes, HUB0, "hub0", center, 0.5f, 0.0f, 2.0f);
        addNode(nodes, HUB1, "hub1", center, -0.5f, 0.0f, 2.0f);
        addNode(nodes, HUB2, "hub2", center, 0.0f, 0.5f, 2.0f);
        addNode(nodes, HUB3, "hub3", center, 0.0f, -0.5f, 2.0f);
        addNode(nodes, COUPLING, "coupling", 0.0f, 0.0f, 1.0f, 5.0f);
        addNode(nodes, ARM, "arm", 0.0f, -0.6f, 1.0f, 5.0f);
        addNode(nodes, ARM2, "arm2", -side, 0.6f, -1.0f, 5.0f);

        WheelContainer wheels = vehicle.wheels;
        wheels.count = 1;
        wheels.nameToIndex.put("W", 0);
        wheels.node1[0] = AXLE_OUTER;
        wheels.node2[0] = AXLE_INNER;
        wheels.wheelDir[0] = mirrored ? -1 : 1;
        wheels.numRays[0] = 2;
        wheels.tireRadius[0] = 0.0f;
        wheels.hubInnerNodes[0] = HUB0;
        wheels.hubOuterNodes[0] = HUB1;
        wheels.hubInnerNodes[1] = HUB2;
        wheels.hubOuterNodes[1] = HUB3;
        return vehicle;
    }

    /** Gives the hub ring a positive forward angular velocity about the axle. */
    private static void spinWheelForward(NodeContainer nodes, float omega) {
        for (int i = HUB0; i <= HUB3; i++) {
            float y = nodes.posY[i];
            float z = nodes.posZ[i];
            nodes.velY[i] = -omega * z;
            nodes.velZ[i] = omega * y;
        }
    }

    private static void integrateForces(NodeContainer nodes, float dt) {
        for (int i = 0; i < nodes.count; i++) {
            if (nodes.mass[i] <= 0.0f) continue;
            nodes.velX[i] += nodes.forceX[i] / nodes.mass[i] * dt;
            nodes.velY[i] += nodes.forceY[i] / nodes.mass[i] * dt;
            nodes.velZ[i] += nodes.forceZ[i] / nodes.mass[i] * dt;
        }
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

    private static double[] netForce(NodeContainer nodes, int... ids) {
        double[] sum = new double[3];
        for (int id : ids) {
            sum[0] += nodes.forceX[id];
            sum[1] += nodes.forceY[id];
            sum[2] += nodes.forceZ[id];
        }
        return sum;
    }

    private static double[] moment(NodeContainer nodes, int... ids) {
        double[] sum = new double[3];
        for (int id : ids) {
            sum[0] += nodes.posY[id] * nodes.forceZ[id] - nodes.posZ[id] * nodes.forceY[id];
            sum[1] += nodes.posZ[id] * nodes.forceX[id] - nodes.posX[id] * nodes.forceZ[id];
            sum[2] += nodes.posX[id] * nodes.forceY[id] - nodes.posY[id] * nodes.forceX[id];
        }
        return sum;
    }

    private static void assertAllForcesZero(NodeContainer nodes, int first, int last) {
        for (int i = first; i <= last; i++) {
            assertForcesZero(nodes, i);
        }
    }

    private static void assertForcesZero(NodeContainer nodes, int... ids) {
        for (int id : ids) {
            assertEquals(0.0f, nodes.forceX[id], 1e-6f, "forceX node " + id);
            assertEquals(0.0f, nodes.forceY[id], 1e-6f, "forceY node " + id);
            assertEquals(0.0f, nodes.forceZ[id], 1e-6f, "forceZ node " + id);
        }
    }
}
