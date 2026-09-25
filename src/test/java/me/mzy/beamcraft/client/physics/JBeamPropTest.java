package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JBeamPropTest {

    @Test
    void parsesDocumentedSteeringWheelPropIntoSharedRenderMeshStream() {
        SoftBodyVehicle vehicle = vehicleWithReferenceNodes();
        JsonArray props = JsonParser.parseString("""
                [
                  ["func", "mesh", "idRef:", "idX:", "idY:", "baseRotation",
                   "rotation", "translation", "min", "max", "offset", "multiplier"],
                  ["steering", "steer_04a", "ref", "x", "y",
                   {"x":-78,"y":0,"z":180}, {"x":0,"y":0,"z":1},
                   {"x":0,"y":0,"z":0}, -1000, 1000, 0, 1,
                   {"baseTranslation":{"x":-0.070,"y":0.605,"z":-0.56},
                    "baseRotationGlobal":{"x":1,"y":2,"z":3}}]
                ]
                """).getAsJsonArray();

        JBeamAssembler.PartEntry entry = new JBeamAssembler.PartEntry(
                new JsonObject(), 3, "body", new JBeamAssembler.TransformContext(), Map.of());
        JBeamParser.parseProps(props, vehicle, "testcar", entry,
                JBeamExpressionEvaluator.contextOf(Map.of()));

        assertEquals(1, vehicle.props.count);
        assertEquals("steering", vehicle.props.function[0]);
        assertEquals(0, vehicle.props.refNode[0]);
        assertEquals(1, vehicle.props.xNode[0]);
        assertEquals(2, vehicle.props.yNode[0]);
        assertEquals(1.0f, vehicle.props.rotationZ[0]);
        assertTrue(vehicle.props.hasBaseTranslation[0]);
        assertEquals(-0.070f, vehicle.props.baseTranslationX[0], 1.0e-6f);
        assertTrue(vehicle.props.hasBaseRotationGlobal[0]);
        assertEquals(2.0f, vehicle.props.baseRotationGlobalY[0], 1.0e-6f);

        assertEquals(1, vehicle.flexbodies.meshCount);
        assertEquals("steer_04a", vehicle.flexbodies.meshName[0]);
        assertEquals(0, vehicle.flexbodies.propIndex[0]);
        assertEquals("testcar", vehicle.flexbodies.vehicleNamespace);
    }

    @Test
    void missingOptionalPropFrameIsSkipped() {
        SoftBodyVehicle vehicle = vehicleWithReferenceNodes();
        JsonArray props = JsonParser.parseString("""
                [
                  ["func", "mesh", "idRef:", "idX:", "idY:"],
                  ["nop", "missing", "ref", "absent", "y", {"optional":true}]
                ]
                """).getAsJsonArray();
        JBeamAssembler.PartEntry entry = new JBeamAssembler.PartEntry(
                new JsonObject(), 0, "body", new JBeamAssembler.TransformContext(), Map.of());

        JBeamParser.parseProps(props, vehicle, "testcar", entry,
                JBeamExpressionEvaluator.contextOf(Map.of()));

        assertEquals(0, vehicle.props.count);
        assertEquals(0, vehicle.flexbodies.meshCount);
    }

    @Test
    void propOrientationMatchesNativeEtk800LiveTransform() {
        float[] x = {0, 2, 1};
        float[] y = {0, 0, 3};
        float[] z = {0, 0, 0};
        float[] basis = new float[9];
        assertTrue(PropContainer.buildReferenceBasis(x, y, z, 0, 1, 2, basis));
        assertEquals(1.0f, basis[0], 1.0e-6f);
        assertEquals(0.0f, basis[3], 1.0e-6f);
        assertEquals(0.0f, basis[1], 1.0e-6f);
        assertEquals(1.0f, basis[4], 1.0e-6f);

        float[] liveX = {0.73017746f, -0.06981215f, 0.73002762f};
        float[] liveY = {0.37811777f, 0.37803710f, 0.65755713f};
        float[] liveZ = {0.67669964f, 0.67691076f, 0.14553191f};
        float[] orientation = new float[9];
        assertTrue(PropContainer.buildPropOrientation(
                liveX, liveY, liveZ, 0, 1, 2,
                -5, 0, 180, 0, 0, 0, orientation));
        float[] expected = {
                0.99999994f, -0.00000905f, -0.00028237f,
                0.00010084f, -0.92221242f, 0.38668233f,
                -0.00026390f, -0.38668233f, -0.92221242f
        };
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], orientation[i], 2.0e-5f, "matrix element " + i);
        }
    }

    @Test
    void propOrientationMatchesNativeVivaceLiveTransform() {
        float[] x = {0.52803898f, 0.52809328f, -0.20791788f};
        float[] y = {0.61747539f, 0.54633522f, 0.54635179f};
        float[] z = {0.09120397f, 0.28514484f, 0.28532347f};
        float[] orientation = new float[9];
        assertTrue(PropContainer.buildPropOrientation(
                x, y, z, 0, 1, 2,
                0, 90, 180, 0, 0, 0, orientation));
        float[] expected = {
                0.99999994f, -0.00010466f, -0.00026291f,
                -0.00000772f, -0.93883097f, 0.34437698f,
                -0.00028287f, -0.34437698f, -0.93883097f
        };
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], orientation[i], 2.0e-5f, "matrix element " + i);
        }
    }

    @Test
    void zeroRotationSunvisorMatchesNativeEtk800LiveTransform() {
        // Body axes sampled alongside getLiveTransformWorld(): ref, back, left, up.
        float[] x = {0.0f, -0.00024250f, 0.99999996f, -0.00012990f};
        float[] y = {0.0f, 0.01817430f, 0.00013429f, 0.99983490f};
        float[] z = {0.0f, -0.99983480f, -0.00024010f, 0.01817430f};
        float[] orientation = new float[9];
        assertTrue(PropContainer.buildGlobalPropOrientation(
                x, y, z, new VehicleCameraData.RefNodes(0, 1, 2, 3),
                -0.031f, -3.382f, -9.177f,
                0, 0, 0, orientation, new float[9]));
        float[] expected = {
                0.98544049f, -0.15972266f, 0.05827282f,
                -0.05604215f, 0.01843887f, 0.99825811f,
                -0.16051891f, -0.98698968f, 0.00921921f
        };
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], orientation[i], 6.0e-4f, "matrix element " + i);
        }
    }

    @Test
    void steeringPropFollowsActualHydroLengthWithNativeNegativeFactorDirection() {
        assertSteeringDirection(-0.2f, 0.9f);
    }

    @Test
    void steeringPropFollowsActualHydroLengthWithNativePositiveFactorDirection() {
        assertSteeringDirection(0.2f, 1.1f);
    }

    private static void assertSteeringDirection(float factor, float displacedLength) {
        SoftBodyVehicle vehicle = vehicleWithReferenceNodes();
        JsonArray hydros = JsonParser.parseString("""
                [
                  ["id1:", "id2:"],
                  ["ref", "x", {"factor":%s, "steeringWheelLock":90}]
                ]
                """.formatted(factor)).getAsJsonArray();
        JBeamAssembler.PartEntry entry = new JBeamAssembler.PartEntry(
                new JsonObject(), 0, "body", new JBeamAssembler.TransformContext(), Map.of());
        JBeamParser.parseHydros(hydros, vehicle, entry);

        JsonArray props = JsonParser.parseString("""
                [
                  ["func", "mesh", "idRef:", "idX:", "idY:", "baseRotation",
                   "rotation", "translation", "min", "max", "offset", "multiplier"],
                  ["steering", "wheel", "ref", "x", "y", {"x":0,"y":0,"z":0},
                   {"x":0,"y":0,"z":1}, {"x":0,"y":0,"z":0}, -1000, 1000, 0, 1]
                ]
                """).getAsJsonArray();
        JBeamParser.parseProps(props, vehicle, "testcar", entry,
                JBeamExpressionEvaluator.contextOf(Map.of()));
        vehicle.props.originBound[0] = true;

        float[] x = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        float[] y = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
        float[] z = new float[6];
        vehicle.electrics.set("steering_input", 1.0);
        vehicle.props.appendRenderNodes(vehicle, x, y, z, vehicle.nodes.count);

        int center = vehicle.nodes.count;
        int axisX = center + 1;
        float restAxisX = x[axisX] - x[center];
        float restAxisY = y[axisX] - y[center];
        float restAxisZ = z[axisX] - z[center];

        // Native BeamNG reports steering=-lock for positive steering input,
        // independently of whether the authored linkage extends or contracts.
        x[1] = displacedLength;
        vehicle.props.appendRenderNodes(vehicle, x, y, z, vehicle.nodes.count);
        float[] expected = new float[9];
        assertTrue(PropContainer.buildPropOrientation(
                x, y, z, 0, 1, 2,
                0, 0, 0, 0, 0, -45, expected));
        assertEquals(expected[0], x[axisX] - x[center], 1.0e-4f);
        assertEquals(expected[3], y[axisX] - y[center], 1.0e-4f);
        assertEquals(expected[6], z[axisX] - z[center], 1.0e-4f);
        float changed = Math.abs((x[axisX] - x[center]) - restAxisX)
                + Math.abs((y[axisX] - y[center]) - restAxisY)
                + Math.abs((z[axisX] - z[center]) - restAxisZ);
        assertTrue(changed > 0.5f, "actual hydro extension must rotate the prop despite raw input being unchanged");
    }

    private static SoftBodyVehicle vehicleWithReferenceNodes() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("ref", 0, 0, 0));
        vehicle.addNode(node("x", 1, 0, 0));
        vehicle.addNode(node("y", 0, 1, 0));
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x, float y, float z) {
        return new PhysicsSpecs.NodeSpec(
                name, x, y, z, 1.0f, 1.0f, 1.0f,
                0, true, false, java.util.List.of());
    }
}
