package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.List;

/** Parses BeamNG internal camera rows into soft-body nodes and support beams. */
public final class JBeamCameraParser {
    private JBeamCameraParser() {
    }

    public static void parseInternal(JsonArray cameras, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        boolean header = true;
        int ordinal = 0;
        float nodeWeight = 1.0f;
        boolean collision = false;
        boolean selfCollision = false;
        float spring = 9000000.0f;
        float damp = 12000.0f;
        float deform = 400000.0f;
        float strength = 1000000.0f;

        for (JsonElement element : cameras) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                nodeWeight = JBeamParser.getFloatSafe(modifier, "nodeWeight", nodeWeight, entry.variables);
                collision = JBeamParser.getBooleanSafe(modifier, "collision", collision);
                selfCollision = JBeamParser.getBooleanSafe(modifier, "selfCollision", selfCollision);
                spring = JBeamParser.getFloatSafe(modifier, "beamSpring", spring, entry.variables);
                damp = JBeamParser.getFloatSafe(modifier, "beamDamp", damp, entry.variables);
                deform = JBeamParser.getFloatSafe(modifier, "beamDeform", deform, entry.variables);
                strength = JBeamParser.getFloatSafe(modifier, "beamStrength", strength, entry.variables);
                continue;
            }
            if (!element.isJsonArray()) {
                continue;
            }

            JsonArray row = element.getAsJsonArray();
            if (header) {
                header = false;
                continue;
            }
            if (row.size() < 5) {
                continue;
            }

            Double rawX = JBeamParser.parseNodeCoordinate(row.get(1), entry.variables);
            Double rawY = JBeamParser.parseNodeCoordinate(row.get(2), entry.variables);
            Double rawZ = JBeamParser.parseNodeCoordinate(row.get(3), entry.variables);
            if (rawX == null || rawY == null || rawZ == null) {
                continue;
            }

            float inlineWeight = nodeWeight;
            boolean inlineCollision = collision;
            boolean inlineSelfCollision = selfCollision;
            float inlineSpring = spring;
            float inlineDamp = damp;
            float inlineDeform = deform;
            float inlineStrength = strength;
            if (row.get(row.size() - 1).isJsonObject()) {
                JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                inlineWeight = JBeamParser.getFloatSafe(inline, "nodeWeight", inlineWeight, entry.variables);
                inlineCollision = JBeamParser.getBooleanSafe(inline, "collision", inlineCollision);
                inlineSelfCollision = JBeamParser.getBooleanSafe(inline, "selfCollision", inlineSelfCollision);
                inlineSpring = JBeamParser.getFloatSafe(inline, "beamSpring", inlineSpring, entry.variables);
                inlineDamp = JBeamParser.getFloatSafe(inline, "beamDamp", inlineDamp, entry.variables);
                inlineDeform = JBeamParser.getFloatSafe(inline, "beamDeform", inlineDeform, entry.variables);
                inlineStrength = JBeamParser.getFloatSafe(inline, "beamStrength", inlineStrength, entry.variables);
            }

            double[] transformed = entry.transform.transformNode(rawX, rawY, rawZ);
            String nodeName = "__beamcraft_camera_" + entry.partId + "_" + ordinal++;
            vehicle.addNode(new PhysicsSpecs.NodeSpec(
                    nodeName,
                    (float) transformed[0],
                    (float) transformed[2],
                    (float) -transformed[1],
                    inlineWeight,
                    0.5f,
                    -1.0f,
                    entry.partId,
                    inlineCollision,
                    inlineSelfCollision,
                    List.of()
            ));
            int nodeIndex = vehicle.nodes.nameToIndex.get(nodeName);

            String type = primitiveString(row.get(0));
            float fov = floatCell(row.get(4), 65.0f, entry);
            vehicle.cameras.addInternal(type, nodeIndex, fov);

            int lastAnchor = Math.min(10, row.size() - 1);
            for (int anchorCell = 5; anchorCell <= lastAnchor; anchorCell++) {
                String anchor = primitiveString(row.get(anchorCell));
                if (anchor == null || anchor.isBlank() || !vehicle.nodes.nameToIndex.containsKey(anchor)) {
                    continue;
                }
                vehicle.addBeam(normalBeam(nodeName, anchor, inlineSpring, inlineDamp, inlineDeform, inlineStrength));
            }
        }
    }

    public static void parseMetadata(JsonObject part, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        if (part.has("refNodes") && part.get("refNodes").isJsonArray()) {
            parseRefNodes(part.getAsJsonArray("refNodes"), vehicle);
        }
        if (part.has("cameraChase") && part.get("cameraChase").isJsonObject()) {
            vehicle.cameras.setChase(parseExternal(part.getAsJsonObject("cameraChase"), entry));
        }
        if (part.has("cameraExternal") && part.get("cameraExternal").isJsonObject()) {
            vehicle.cameras.setExternal(parseExternal(part.getAsJsonObject("cameraExternal"), entry));
        }
    }

    private static void parseRefNodes(JsonArray rows, SoftBodyVehicle vehicle) {
        boolean header = true;
        for (JsonElement element : rows) {
            if (!element.isJsonArray()) {
                continue;
            }
            if (header) {
                header = false;
                continue;
            }
            JsonArray row = element.getAsJsonArray();
            if (row.size() < 4) {
                continue;
            }
            Integer ref = nodeIndex(row.get(0), vehicle);
            Integer back = nodeIndex(row.get(1), vehicle);
            Integer left = nodeIndex(row.get(2), vehicle);
            Integer up = nodeIndex(row.get(3), vehicle);
            if (ref != null && back != null && left != null && up != null) {
                vehicle.cameras.setRefNodes(new VehicleCameraData.RefNodes(ref, back, left, up));
                return;
            }
        }
    }

    private static VehicleCameraData.ExternalCamera parseExternal(JsonObject object, JBeamAssembler.PartEntry entry) {
        float distance = JBeamParser.getFloatSafe(object, "distance", 5.0f, entry.variables);
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        if (object.has("offset") && object.get("offset").isJsonObject()) {
            JsonObject offset = object.getAsJsonObject("offset");
            x = JBeamParser.getFloatSafe(offset, "x", 0.0f, entry.variables);
            float beamY = JBeamParser.getFloatSafe(offset, "y", 0.0f, entry.variables);
            float beamZ = JBeamParser.getFloatSafe(offset, "z", 0.0f, entry.variables);
            y = beamZ;
            z = -beamY;
        }
        return new VehicleCameraData.ExternalCamera(x, y, z, distance);
    }

    private static Integer nodeIndex(JsonElement element, SoftBodyVehicle vehicle) {
        String name = primitiveString(element);
        return name == null ? null : vehicle.nodes.nameToIndex.get(name);
    }

    private static String primitiveString(JsonElement element) {
        return element != null && element.isJsonPrimitive() ? element.getAsString() : null;
    }

    private static float floatCell(JsonElement element, float fallback, JBeamAssembler.PartEntry entry) {
        Double value = JBeamParser.parseNodeCoordinate(element, entry.variables);
        return value == null ? fallback : value.floatValue();
    }

    private static PhysicsSpecs.BeamSpec normalBeam(
            String cameraNode, String anchor, float spring, float damp, float deform, float strength) {
        return new PhysicsSpecs.BeamSpec(
                BeamContainer.BEAM_NORMAL, cameraNode, anchor, null,
                List.of(), 0, false,
                spring, damp, deform, strength,
                1.0f, 0.0f, 0.0f,
                1.0f, 1.0f, -1.0f, -1.0f,
                spring, damp, -1.0f, -1.0f,
                -1.0f, -1.0f, spring, damp, 0.0f,
                PhysicsWorld.KINDA_BIG_NUMBER
        );
    }
}
