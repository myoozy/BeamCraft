package me.mzy.beamcraft.client.input;

import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricValues;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class DriverInputFilterTest {
    @Test
    void advancesLatchedKeyboardTargetAtPhysicsSubstepRate() {
        ElectricBus bus = new ElectricBus();
        DriverInputFilter filter = new DriverInputFilter(bus);
        filter.configure(0.10, 0.20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        bus.set(ElectricSignals.STEERING_INPUT, 1.0);
        filter.latchTargets(bus.snapshot());

        assertEquals(0.25, filter.update(0.025f, bus.snapshot())
                .get(bus.signalId(ElectricSignals.STEERING_INPUT)), 0.0001);
        assertEquals(0.50, filter.update(0.025f, bus.snapshot())
                .get(bus.signalId(ElectricSignals.STEERING_INPUT)), 0.0001);
        assertEquals(1.00, filter.update(0.050f, bus.snapshot())
                .get(bus.signalId(ElectricSignals.STEERING_INPUT)), 0.0001);
    }

    @Test
    void ignoresContinuousTargetChangesUntilNextPreparedStepLatch() {
        ElectricBus bus = new ElectricBus();
        DriverInputFilter filter = new DriverInputFilter(bus);
        filter.configure(0.10, 0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        bus.set(ElectricSignals.STEERING_INPUT, 1.0);
        filter.latchTargets(bus.snapshot());
        assertEquals(0.5, steering(filter.update(0.05f, bus.snapshot()), bus), 0.0001);

        bus.set(ElectricSignals.STEERING_INPUT, -1.0);
        assertEquals(1.0, steering(filter.update(0.05f, bus.snapshot()), bus), 0.0001,
                "the current 50 ms physics block must keep its original target");

        filter.latchTargets(bus.snapshot());
        assertEquals(0.5, steering(filter.update(0.05f, bus.snapshot()), bus), 0.0001);
    }

    @Test
    void zeroTimeIsDirectAndUnrelatedSignalsStillUseLatestSnapshot() {
        ElectricBus bus = new ElectricBus();
        DriverInputFilter filter = new DriverInputFilter(bus);
        filter.configure(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int customSignal = bus.register("custom");

        bus.set(ElectricSignals.THROTTLE_INPUT, 1.0);
        filter.latchTargets(bus.snapshot());
        bus.set("custom", 0.75);
        ElectricValues values = filter.update(0.0005f, bus.snapshot());

        assertEquals(1.0, values.get(bus.signalId(ElectricSignals.THROTTLE_INPUT)), 0.0001);
        assertEquals(0.75, values.get(customSignal), 0.0001);
    }

    private static double steering(ElectricValues values, ElectricBus bus) {
        return values.get(bus.signalId(ElectricSignals.STEERING_INPUT));
    }
}
