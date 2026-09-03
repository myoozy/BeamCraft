package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class WheelBrakeTest {
    @Test
    void splitCurveReducesTorqueAboveConfiguredInputPoint() {
        assertEquals(750.0f, WheelContainer.calculateServiceBrakeTorque(1000.0f, 1.0f, 0.5f, 0.5f));
        assertEquals(250.0f, WheelContainer.calculateServiceBrakeTorque(1000.0f, 0.25f, 0.5f, 0.5f));
    }

    @Test
    void pressureStateUsesSeparateApplyAndReleaseDelays() {
        WheelContainer wheels = new WheelContainer(new SoftBodyVehicle(null));
        wheels.count = 1;
        wheels.brakeTorque[0] = 1000.0f;
        wheels.brakeInputSplit[0] = 0.5f;
        wheels.brakeSplitCoef[0] = 0.5f;
        wheels.brakePressureInDelay[0] = 0.1f;
        wheels.brakePressureOutDelay[0] = 0.2f;

        wheels.applyServiceBrakes(1.0f, 0.05f);
        assertEquals(500.0f, wheels.serviceBrakeTorque[0]);
        wheels.applyServiceBrakes(1.0f, 0.05f);
        assertEquals(750.0f, wheels.serviceBrakeTorque[0]);

        wheels.applyServiceBrakes(0.0f, 0.05f);
        assertEquals(500.0f, wheels.serviceBrakeTorque[0]);
    }

    @Test
    void brakeTorqueOpposesRotationWithoutReversingTheWheel() {
        SoftBodyVehicle vehicle = rotatingWheel();
        WheelContainer wheels = vehicle.wheels;
        float before = wheels.getAngularVelocity(0);

        wheels.applyServiceBrakes(1.0f, 0.01f);
        integrateForces(vehicle.nodes, 0.01f);

        float after = wheels.getAngularVelocity(0);
        assertTrue(before > 0.0f);
        assertTrue(after >= -1.0e-5f, "braking must not reverse a nearly stopped wheel");
        assertTrue(after < before, "service brake must remove wheel angular velocity");
    }

    @Test
    void parkingBrakeUsesItsOwnTorqueThroughTheSharedCompliantConstraint() {
        SoftBodyVehicle vehicle = rotatingWheel();
        WheelContainer wheels = vehicle.wheels;
        wheels.brakeTorque[0] = 0.0f;
        wheels.parkingTorque[0] = 1000.0f;
        float before = wheels.getAngularVelocity(0);

        wheels.applyBrakes(0.0f, 1.0f, 0.01f);
        integrateForces(vehicle.nodes, 0.01f);

        assertTrue(wheels.brakeAngle[0] > 0.0f);
        assertTrue(wheels.getAngularVelocity(0) < before);
    }

    @Test
    void brakeSpringControlsHowQuicklyConstraintTorqueBuilds() {
        SoftBodyVehicle softVehicle = rotatingWheel();
        SoftBodyVehicle stiffVehicle = rotatingWheel();
        softVehicle.wheels.brakeSpring[0] = 0.1f;
        stiffVehicle.wheels.brakeSpring[0] = 10.0f;

        softVehicle.wheels.applyServiceBrakes(1.0f, 0.01f);
        stiffVehicle.wheels.applyServiceBrakes(1.0f, 0.01f);
        integrateForces(softVehicle.nodes, 0.01f);
        integrateForces(stiffVehicle.nodes, 0.01f);

        assertTrue(stiffVehicle.wheels.getAngularVelocity(0)
                        < softVehicle.wheels.getAngularVelocity(0),
                "higher brakeSpring must build more constraint torque for the same angular travel");
    }

    @Test
    void brakeSpringAngleStopsAccumulatingWhileTheBrakeIsSlipping() {
        SoftBodyVehicle vehicle = rotatingWheel();
        WheelContainer wheels = vehicle.wheels;

        wheels.applyServiceBrakes(1.0f, 0.01f);
        float saturatedAngle = wheels.brakeAngle[0];
        wheels.applyServiceBrakes(1.0f, 0.01f);

        assertEquals(1.0f / wheels.brakeSpring[0], saturatedAngle, 1.0e-6f);
        assertEquals(saturatedAngle, wheels.brakeAngle[0], 1.0e-6f,
                "slipping must not keep winding up the compliant brake constraint");
    }

    private static SoftBodyVehicle rotatingWheel() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        nodes.count = 6;
        nodes.posX[0] = 0.0f;
        nodes.posX[1] = 1.0f;
        nodes.mass[0] = nodes.mass[1] = 1.0f;

        float[] x = {0.4f, 0.6f, 0.4f, 0.6f};
        float[] y = {1.0f, 1.0f, -1.0f, -1.0f};
        for (int i = 0; i < 4; i++) {
            int node = i + 2;
            nodes.posX[node] = x[i];
            nodes.posY[node] = y[i];
            nodes.velZ[node] = y[i] * 10.0f;
            nodes.mass[node] = 1.0f;
        }

        WheelContainer wheels = vehicle.wheels;
        wheels.count = 1;
        wheels.node1[0] = 1;
        wheels.node2[0] = 0;
        wheels.wheelDir[0] = 1;
        wheels.numRays[0] = 2;
        wheels.hubInnerNodes[0] = 2;
        wheels.hubOuterNodes[0] = 3;
        wheels.hubInnerNodes[1] = 4;
        wheels.hubOuterNodes[1] = 5;
        wheels.brakeTorque[0] = 10_000.0f;
        wheels.parkingTorque[0] = 0.0f;
        wheels.brakeSpring[0] = 10.0f;
        wheels.brakeInputSplit[0] = 1.0f;
        wheels.brakeSplitCoef[0] = 1.0f;
        return vehicle;
    }

    private static void integrateForces(NodeContainer nodes, float dt) {
        for (int i = 0; i < nodes.count; i++) {
            if (nodes.mass[i] <= 0.0f) continue;
            nodes.velX[i] += nodes.forceX[i] / nodes.mass[i] * dt;
            nodes.velY[i] += nodes.forceY[i] / nodes.mass[i] * dt;
            nodes.velZ[i] += nodes.forceZ[i] / nodes.mass[i] * dt;
        }
    }
}
