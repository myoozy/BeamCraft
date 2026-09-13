package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Body pitch/roll read off the authored {@code refNodes} triple.
 *
 * <p>The sign conventions are load-bearing: the point of this readout is to be
 * compared against BeamNG's pitch/roll display, so a flipped sign would send the
 * investigation the wrong way. Pitch is positive nose-up, roll positive left-up.
 */
class BodyAttitudeTest {

    private static final float EPS = 1.0e-3f;

    @Test
    void levelBodyReadsZeroPitchAndRoll() {
        SoftBodyVehicle vehicle = rig(0.0f, 0.0f);
        float[] out = new float[2];
        assertTrue(vehicle.bodyAttitudeDeg(out));
        assertEquals(0.0f, out[0], EPS);
        assertEquals(0.0f, out[1], EPS);
    }

    @Test
    void noseUpIsPositivePitch() {
        // Ref lifted above back by 10 degrees over a unit lever.
        SoftBodyVehicle vehicle = rig(10.0f, 0.0f);
        float[] out = new float[2];
        assertTrue(vehicle.bodyAttitudeDeg(out));
        assertEquals(10.0f, out[0], EPS, "nose up must read positive");
        assertEquals(0.0f, out[1], EPS);
    }

    @Test
    void noseDownIsNegativePitch() {
        SoftBodyVehicle vehicle = rig(-7.5f, 0.0f);
        float[] out = new float[2];
        assertTrue(vehicle.bodyAttitudeDeg(out));
        assertEquals(-7.5f, out[0], EPS, "nose down must read negative");
    }

    @Test
    void leftSideUpIsPositiveRoll() {
        SoftBodyVehicle vehicle = rig(0.0f, 6.0f);
        float[] out = new float[2];
        assertTrue(vehicle.bodyAttitudeDeg(out));
        assertEquals(0.0f, out[0], EPS);
        assertEquals(6.0f, out[1], EPS, "left side up must read positive");
    }

    @Test
    void missingOrDegenerateRefNodesReportUnavailable() {
        SoftBodyVehicle noRefs = rig(0.0f, 0.0f);
        noRefs.cameras.clear();
        float[] out = {123.0f, 456.0f};
        assertFalse(noRefs.bodyAttitudeDeg(out));
        assertEquals(123.0f, out[0], 0.0f, "an unavailable reading must not touch the output");

        // A collapsed ref->back lever has no direction to measure.
        SoftBodyVehicle collapsed = rig(0.0f, 0.0f);
        collapsed.nodes.posX[1] = collapsed.nodes.posX[0];
        collapsed.nodes.posY[1] = collapsed.nodes.posY[0];
        collapsed.nodes.posZ[1] = collapsed.nodes.posZ[0];
        assertFalse(collapsed.bodyAttitudeDeg(new float[2]));
    }

    /**
     * A rig with the ref/back/left triple at unit distances: {@code back} one unit
     * behind {@code ref} along +z, {@code left} one unit to the +x side, then the whole
     * assembly tilted by the requested pitch and roll in degrees.
     */
    private static SoftBodyVehicle rig(float pitchDeg, float rollDeg) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("ref", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(node("back", 0.0f, 0.0f, 1.0f));
        vehicle.addNode(node("left", 1.0f, 0.0f, 0.0f));
        vehicle.finalizePhysicsSetup();
        vehicle.cameras.setRefNodes(new VehicleCameraData.RefNodes(0, 1, 2, -1));

        double pitch = Math.toRadians(pitchDeg);
        double roll = Math.toRadians(rollDeg);
        // Nose-up is a rotation about the lateral (+x) axis; left-up about the
        // longitudinal axis. Both are applied about the ref node at the origin.
        for (int i = 1; i < 3; i++) {
            double x = vehicle.nodes.posX[i];
            double y = vehicle.nodes.posY[i];
            double z = vehicle.nodes.posZ[i];
            double py = y * Math.cos(pitch) - z * Math.sin(pitch);
            double pz = y * Math.sin(pitch) + z * Math.cos(pitch);
            y = py;
            z = pz;
            double rx = x * Math.cos(roll) - y * Math.sin(roll);
            double ry = x * Math.sin(roll) + y * Math.cos(roll);
            vehicle.nodes.posX[i] = (float) rx;
            vehicle.nodes.posY[i] = (float) ry;
            vehicle.nodes.posZ[i] = (float) z;
        }
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x, float y, float z) {
        return new PhysicsSpecs.NodeSpec(name, x, y, z, 1.0f, 1.0f, 1.0f, 0, false, false, List.of());
    }
}
