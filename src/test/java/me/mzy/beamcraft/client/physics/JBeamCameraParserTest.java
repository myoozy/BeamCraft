package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class JBeamCameraParserTest {
    @Test
    void createsPhysicalCameraNodeAndSixBeams() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        for (int i = 1; i <= 6; i++) {
            vehicle.addNode(new PhysicsSpecs.NodeSpec(
                    "n" + i, i, 0.0f, 0.0f, 10.0f,
                    0.5f, -1.0f, 0, false, false, List.of()));
        }
        JsonArray cameras = JsonParser.parseString("""
                [
                  ["type", "x", "y", "z", "fov", "id1:", "id2:", "id3:", "id4:", "id5:", "id6:"],
                  {"nodeWeight":1.24,"beamSpring":46000,"beamDamp":435},
                  ["hood",0.0,-0.64,1.2,65,"n1","n2","n3","n4","n5","n6"],
                  ["driver",0.2,0.1,1.1,55,"n1","n2","n3","n4","n5","n6"]
                ]
                """).getAsJsonArray();
        JBeamAssembler.PartEntry entry = new JBeamAssembler.PartEntry(
                new JsonObject(), 7, "body", new JBeamAssembler.TransformContext(), new HashMap<>());

        JBeamCameraParser.parseInternal(cameras, vehicle, entry);

        assertEquals(8, vehicle.nodes.count);
        assertEquals(12, vehicle.normalBeams.count);
        VehicleCameraData.InternalCamera preferred = vehicle.cameras.preferredInternal();
        assertNotNull(preferred);
        assertEquals("driver", preferred.type());
        assertEquals(55.0f, preferred.fov());
    }

    @Test
    void readsRefNodesAndExteriorCameraMetadata() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        for (String name : List.of("ref", "back", "left", "up")) {
            vehicle.addNode(new PhysicsSpecs.NodeSpec(
                    name, 0.0f, 0.0f, 0.0f, 1.0f,
                    0.5f, -1.0f, 0, false, false, List.of()));
        }
        JsonObject part = JsonParser.parseString("""
                {
                  "refNodes":[
                    ["ref:","back:","left:","up:"],
                    ["ref","back","left","up"]
                  ],
                  "cameraExternal":{
                    "distance":7.5,
                    "offset":{"x":1.0,"y":2.0,"z":3.0}
                  }
                }
                """).getAsJsonObject();
        JBeamAssembler.PartEntry entry = new JBeamAssembler.PartEntry(
                part, 0, "body", new JBeamAssembler.TransformContext(), new HashMap<>());

        JBeamCameraParser.parseMetadata(part, vehicle, entry);

        assertNotNull(vehicle.cameras.refNodes());
        VehicleCameraData.ExternalCamera external = vehicle.cameras.exteriorFallback();
        assertNotNull(external);
        assertEquals(7.5f, external.distance());
        assertEquals(1.0f, external.offsetX());
        assertEquals(3.0f, external.offsetY());
        assertEquals(-2.0f, external.offsetZ());
    }
}
