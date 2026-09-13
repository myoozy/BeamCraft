package me.mzy.beamcraft.client.physics.electrics;

import me.mzy.beamcraft.client.physics.PhysicsWorld;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ElectricBusTest {
    @Test
    void registeredIdsRemainStableAndUnknownSignalsReadAsZero() {
        ElectricBus bus = new ElectricBus();

        int steering = bus.register(ElectricSignals.STEERING_INPUT);
        int throttle = bus.register(ElectricSignals.THROTTLE_INPUT);

        assertEquals(steering, bus.register(ElectricSignals.STEERING_INPUT));
        assertEquals(0, steering);
        assertEquals(1, throttle);
        assertEquals(-1, bus.signalId("missing"));
        assertEquals(0.0, bus.get("missing"));
    }

    @Test
    void snapshotDoesNotChangeWhenMutableBusChanges() {
        ElectricBus bus = new ElectricBus();
        int steering = bus.register(ElectricSignals.STEERING_INPUT);
        bus.set(steering, 0.25);
        ElectricSnapshot first = bus.snapshot();

        bus.set(steering, -0.75);
        bus.set("custom", 1.0);
        ElectricSnapshot second = bus.snapshot();

        assertEquals(0.25, first.get(steering));
        assertFalse(first.contains("custom"));
        assertEquals(-0.75, second.get(ElectricSignals.STEERING_INPUT));
        assertTrue(second.contains("custom"));
        assertTrue(second.revision() > first.revision());
    }

    @Test
    void unchangedBusReusesItsImmutableSnapshot() {
        ElectricBus bus = new ElectricBus();
        bus.set("actuator", 0.25);

        ElectricSnapshot first = bus.snapshot();

        assertSame(first, bus.snapshot());
    }

    @Test
    void preparedPhysicsStepCapturesEachVehiclesInitialSignals() {
        PhysicsWorld world = new PhysicsWorld();
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        world.addVehicle(vehicle);
        vehicle.electrics.set("actuator", 0.25);

        PhysicsWorld.PreparedStep prepared = world.prepareStep(null, 0.001);
        vehicle.electrics.set("actuator", 0.75);

        assertEquals(0.25, prepared.electricSnapshots().getFirst().get("actuator"));
        assertEquals(0.75, vehicle.electrics.get("actuator"));
    }

    @Test
    void physicsRefreshesElectricSnapshotEveryTenSubsteps() {
        PhysicsWorld world = new PhysicsWorld();
        RecordingVehicle vehicle = new RecordingVehicle();
        world.addVehicle(vehicle);
        vehicle.electrics.set("actuator", 0.0);

        PhysicsWorld.PreparedStep prepared = world.prepareStep(null, 0.01);
        world.simulatePreparedStep(prepared);

        assertEquals(20, vehicle.observedValues.size());
        assertEquals(List.of(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                vehicle.observedValues.subList(0, 10));
        assertEquals(List.of(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                vehicle.observedValues.subList(10, 20));
    }

    @Test
    void resetKeepsIdsButClearsValues() {
        ElectricBus bus = new ElectricBus();
        int signal = bus.register("door");
        bus.set(signal, 1.0);

        bus.resetValues();

        assertEquals(signal, bus.signalId("door"));
        assertEquals(0.0, bus.get(signal));
    }

    @Test
    void rejectsInvalidNamesValuesAndIds() {
        ElectricBus bus = new ElectricBus();

        assertThrows(IllegalArgumentException.class, () -> bus.register(" "));
        assertThrows(IllegalArgumentException.class, () -> bus.set("bad", Double.NaN));
        assertEquals(-1, bus.signalId("bad"));
        assertThrows(IndexOutOfBoundsException.class, () -> bus.set(4, 1.0));
    }

    private static final class RecordingVehicle extends SoftBodyVehicle {
        private final List<Double> observedValues = new ArrayList<>();

        private RecordingVehicle() {
            super(null);
        }

        @Override
        public void solveInternalForces(float dt, float plasticRelaxation, ElectricSnapshot snapshot) {
            observedValues.add(snapshot.get("actuator"));
            if (observedValues.size() == PhysicsWorld.ELECTRIC_SNAPSHOT_SUBSTEP_INTERVAL) {
                electrics.set("actuator", 1.0);
            }
        }
    }
}
