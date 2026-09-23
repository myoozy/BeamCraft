package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsWorldLifecycleTest {

    @Test
    void clearReleasesEveryOwnedVehicleBeforeDroppingTheList() {
        PhysicsWorld world = new PhysicsWorld();
        TrackingVehicle first = new TrackingVehicle();
        TrackingVehicle second = new TrackingVehicle();
        world.addVehicle(first);
        world.addVehicle(second);

        world.clear();

        assertTrue(first.cleared);
        assertTrue(second.cleared);
        assertEquals(0, world.vehicles.size());
    }

    private static final class TrackingVehicle extends SoftBodyVehicle {
        private boolean cleared;

        private TrackingVehicle() {
            super(null);
        }

        @Override
        public void clear() {
            cleared = true;
            super.clear();
        }
    }
}
