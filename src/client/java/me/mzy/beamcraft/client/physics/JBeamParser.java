package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

/**
 * Parses JBeam formatted JSON data into physics world elements
 * Provides safe value extraction and handles all JBeam component types
 */
public class JBeamParser {
    public static final java.util.Map<String, String[]> RAIL_MAP = new java.util.HashMap<>();

    public static double getDoubleSafe(JsonObject obj, String key, double defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;

        String str = el.getAsString().trim();
        if (str.isEmpty()) return defaultValue;

        if (str.contains("FLT_MAX") || str.contains("MAX_FLT")) return PhysicsWorld.KINDA_BIG_NUMBER;
        if (str.contains("FLT_MIN") || str.contains("MIN_FLT")) return PhysicsWorld.KINDA_SMALL_NUMBER;

        try {
            return Double.parseDouble(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static String getStringSafe(JsonObject obj, String key, String defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;
        return el.getAsString().trim();
    }

    public static boolean getBooleanSafe(JsonObject obj, String key, boolean defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;
        return el.getAsBoolean();
    }

    public static java.util.List<String> parseGroups(JsonElement el) {
        java.util.List<String> list = new java.util.ArrayList<>();
        if (el == null || el.isJsonNull()) return list;

        if (el.isJsonPrimitive()) {
            String g = el.getAsString().trim();
            if (!g.isEmpty()) {
                list.add(g);
            }
        } else if (el.isJsonArray()) {
            for (JsonElement item : el.getAsJsonArray()) {
                if (item.isJsonPrimitive()) {
                    String g = item.getAsString().trim();
                    if (!g.isEmpty()) list.add(g);
                }
            }
        }
        return list;
    }

    // --- 1. Node Parsing ---
    public static void parseNodes(JsonArray nodes, SoftBodyVehicle vehicle, int partId, CouplerRegistry couplerRegistry, JBeamAssembler.TransformContext transform) {
        boolean isHeader = true;

        double currentWeight = 50.0;
        double currentFriction = 0.5;
        double currentSlidingFriction = -1;
        boolean currentCollision = true;
        boolean currentSelfCollision = false;

        java.util.List<String> currentGroups = new java.util.ArrayList<>();

        for (JsonElement element : nodes) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentWeight = getDoubleSafe(modifier, "nodeWeight", currentWeight);
                currentFriction = getDoubleSafe(modifier, "frictionCoef", currentFriction);
                currentSlidingFriction = getDoubleSafe(modifier, "slidingFrictionCoef", currentSlidingFriction);
                currentCollision = getBooleanSafe(modifier, "collision", currentCollision);
                currentSelfCollision = getBooleanSafe(modifier, "selfCollision", currentSelfCollision);

                if (modifier.has("group")) {
                    currentGroups = parseGroups(modifier.get("group"));
                }
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() < 4) continue;

                double inlineWeight = currentWeight;
                double inlineFriction = currentFriction;
                double inlineSlidingFriction = currentSlidingFriction;
                boolean inlineCollision = currentCollision;
                boolean inlineSelfCollision = currentSelfCollision;

                java.util.List<String> inlineGroups = currentGroups;

                String inlineTag = "";
                String inlineCouplerTag = "";
                double inlineStartRadius = 0.25;
                double inlineCouplerStrength = PhysicsWorld.KINDA_BIG_NUMBER;
                boolean inlineCouplerWeld = false;
                double inlineCouplerLatchSpeed = 0.3;
                double inlineCouplerLockRadius = 0.025;

                if (row.get(row.size() - 1).isJsonObject()) {
                    JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                    inlineWeight = getDoubleSafe(inline, "nodeWeight", inlineWeight);
                    inlineFriction = getDoubleSafe(inline, "frictionCoef", inlineFriction);
                    inlineSlidingFriction = getDoubleSafe(inline, "slidingFrictionCoef", inlineSlidingFriction);
                    inlineCollision = getBooleanSafe(inline, "collision", inlineCollision);
                    inlineSelfCollision = getBooleanSafe(inline, "selfCollision", inlineSelfCollision);

                    if (inline.has("group")) {
                        inlineGroups = parseGroups(inline.get("group"));
                    }

                    inlineTag = getStringSafe(inline, "tag", inlineTag);
                    inlineCouplerTag = getStringSafe(inline, "couplerTag", inlineCouplerTag);
                    inlineStartRadius = getDoubleSafe(inline, "couplerStartRadius", inlineStartRadius);
                    inlineCouplerStrength = getDoubleSafe(inline, "couplerStrength", inlineCouplerStrength);
                    if (inline.has("couplerWeld")) inlineCouplerWeld = getBooleanSafe(inline, "couplerWeld", inlineCouplerWeld);
                    else if (inline.has("couplerLock")) inlineCouplerWeld = getBooleanSafe(inline, "couplerLock", inlineCouplerWeld);
                    inlineCouplerLatchSpeed = getDoubleSafe(inline, "couplerLatchSpeed", inlineCouplerLatchSpeed);
                    inlineCouplerLockRadius = getDoubleSafe(inline, "couplerLockRadius", inlineCouplerLockRadius);
                }

                String id = row.get(0).getAsString();
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;

                try {
                    // 提取原始 BeamNG 空间位置
                    double rawX = row.get(1).getAsDouble();
                    double rawY = row.get(2).getAsDouble();
                    double rawZ = row.get(3).getAsDouble();

                    // 应用插槽级联变换处理逻辑 (包含对称镜像平移、欧拉角旋转、绝对位移)
                    double[] transformed = transform.transformNode(rawX, rawY, rawZ);

                    // 最终引擎空间转换: flip X, swap Y and Z
                    x = +transformed[0];
                    y = +transformed[2];
                    z = -transformed[1];
                } catch (Exception e) {
                    System.err.println("⚠️ Failed to parse node coordinates, skipping: " + id);
                    continue;
                }

                if (!inlineTag.isEmpty() || !inlineCouplerTag.isEmpty()) {
                    couplerRegistry.register(id, inlineTag, inlineCouplerTag, inlineStartRadius, inlineCouplerLatchSpeed, inlineCouplerStrength, inlineCouplerWeld, inlineCouplerLockRadius);
                }

                vehicle.addNode(id, x, y, z, inlineWeight, inlineFriction, inlineSlidingFriction, partId, inlineCollision, inlineSelfCollision, inlineGroups);
            }
        }
    }

    // --- 2. Beam Parsing ---
    public static void parseBeams(JsonArray beams, SoftBodyVehicle vehicle, int partId) {
        boolean isHeader = true;
        int currentType = BeamContainer.BEAM_NORMAL;

        double currentPrecomp = 1.0, currentPrecompRange = 0.0, currentPrecompTime = 0.0;
        double currentSpring = 9000000.0, currentDamp = 12000.0;
        double currentDeform = 400000.0, currentStrength = 1000000.0;

        double currentShortBound = 1.0, currentLongBound = 1.0;
        double currentShortBoundRange = -1.0, currentLongBoundRange = -1.0;
        double currentLimitSpring = currentSpring, currentLimitDamp = currentDamp;

        double currentDampVelSplit = -1.0, currentDampFast = -1.0;
        double currentDampRebound = -1.0, currentDampReboundFast = -1.0;

        double currentSpringExpansion = currentSpring, currentDampExpansion = currentDamp;
        double currentTransitionZone = 0.0;

        for (JsonElement element : beams) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                String bt = getStringSafe(modifier, "beamType", "");
                if (!bt.isEmpty()) {
                    if (bt.equals("|NORMAL")) currentType = BeamContainer.BEAM_NORMAL;
                    else if (bt.equals("|SUPPORT")) currentType = BeamContainer.BEAM_SUPPORT;
                    else if (bt.equals("|BOUNDED")) currentType = BeamContainer.BEAM_BOUNDED;
                    else if (bt.equals("|LBEAM")) currentType = BeamContainer.BEAM_LBEAM;
                    else if (bt.equals("|HYDRO")) currentType = BeamContainer.BEAM_HYDRO;
                    else if (bt.equals("|ANISOTROPIC")) currentType = BeamContainer.BEAM_ANISOTROPIC;
                }

                currentPrecomp = getDoubleSafe(modifier, "beamPrecompression", currentPrecomp);
                currentPrecompRange = getDoubleSafe(modifier, "precompressionRange", currentPrecompRange);
                currentPrecompTime = getDoubleSafe(modifier, "beamPrecompressionTime", currentPrecompTime);
                currentSpring = getDoubleSafe(modifier, "beamSpring", currentSpring);
                currentDamp = getDoubleSafe(modifier, "beamDamp", currentDamp);
                currentDeform = getDoubleSafe(modifier, "beamDeform", currentDeform);
                currentStrength = getDoubleSafe(modifier, "beamStrength", currentStrength);

                currentShortBound = getDoubleSafe(modifier, "beamShortBound", currentShortBound);
                currentLongBound = getDoubleSafe(modifier, "beamLongBound", currentLongBound);
                currentShortBoundRange = getDoubleSafe(modifier, "shortBoundRange", currentShortBoundRange);
                currentLongBoundRange = getDoubleSafe(modifier, "longBoundRange", currentLongBoundRange);
                currentLimitSpring = getDoubleSafe(modifier, "beamLimitSpring", currentLimitSpring);
                currentLimitDamp = getDoubleSafe(modifier, "beamLimitDamp", currentLimitDamp);

                currentDampVelSplit = getDoubleSafe(modifier, "beamDampVelocitySplit", currentDampVelSplit);
                currentDampFast = getDoubleSafe(modifier, "beamDampFast", currentDampFast);
                currentDampRebound = getDoubleSafe(modifier, "beamDampRebound", currentDampRebound);
                currentDampReboundFast = getDoubleSafe(modifier, "beamDampReboundFast", currentDampReboundFast);

                currentSpringExpansion = getDoubleSafe(modifier, "springExpansion", currentSpringExpansion);
                currentDampExpansion = getDoubleSafe(modifier, "dampExpansion", currentDampExpansion);
                currentTransitionZone = getDoubleSafe(modifier, "transitionZone", currentTransitionZone);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {
                    int inlineType = currentType;
                    double inlineSpring = currentSpring, inlineDamp = currentDamp;
                    double inlineDeform = currentDeform, inlineStrength = currentStrength;
                    double inlinePrecomp = currentPrecomp, inlinePrecompRange = currentPrecompRange, inlinePrecompTime = currentPrecompTime;
                    double inlineShortBound = currentShortBound, inlineLongBound = currentLongBound;
                    double inlineShortBoundRange = currentShortBoundRange, inlineLongBoundRange = currentLongBoundRange;
                    double inlineLimitS = currentLimitSpring, inlineLimitD = currentLimitDamp;
                    double inlineDampVelSplit = currentDampVelSplit, inlineDampFast = currentDampFast;
                    double inlineDampRebound = currentDampRebound, inlineDampReboundFast = currentDampReboundFast;
                    double inlineSpringExpansion = currentSpringExpansion, inlineDampExpansion = currentDampExpansion;
                    double inlineTransitionZone = currentTransitionZone;
                    String inlineId3 = null;

                    if (row.size() >= 3 && row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();

                        inlineSpring = getDoubleSafe(inline, "beamSpring", inlineSpring);
                        inlineDamp = getDoubleSafe(inline, "beamDamp", inlineDamp);
                        inlineDeform = getDoubleSafe(inline, "beamDeform", inlineDeform);
                        inlineStrength = getDoubleSafe(inline, "beamStrength", inlineStrength);
                        inlinePrecomp = getDoubleSafe(inline, "beamPrecompression", inlinePrecomp);
                        inlinePrecompRange = getDoubleSafe(inline, "precompressionRange", inlinePrecompRange);
                        inlinePrecompTime = getDoubleSafe(inline, "beamPrecompressionTime", inlinePrecompTime);

                        inlineShortBound = getDoubleSafe(inline, "beamShortBound", inlineShortBound);
                        inlineLongBound = getDoubleSafe(inline, "beamLongBound", inlineLongBound);
                        inlineShortBoundRange = getDoubleSafe(inline, "shortBoundRange", inlineShortBoundRange);
                        inlineLongBoundRange = getDoubleSafe(inline, "longBoundRange", inlineLongBoundRange);
                        inlineLimitS = getDoubleSafe(inline, "beamLimitSpring", inlineLimitS);
                        inlineLimitD = getDoubleSafe(inline, "beamLimitDamp", inlineLimitD);

                        inlineDampVelSplit = getDoubleSafe(inline, "beamDampVelocitySplit", inlineDampVelSplit);
                        inlineDampFast = getDoubleSafe(inline, "beamDampFast", inlineDampFast);
                        inlineDampRebound = getDoubleSafe(inline, "beamDampRebound", inlineDampRebound);
                        inlineDampReboundFast = getDoubleSafe(inline, "beamDampReboundFast", inlineDampReboundFast);

                        inlineSpringExpansion = getDoubleSafe(inline, "springExpansion", inlineSpringExpansion);
                        inlineDampExpansion = getDoubleSafe(inline, "dampExpansion", inlineDampExpansion);
                        inlineTransitionZone = getDoubleSafe(inline, "transitionZone", inlineTransitionZone);

                        String bt = getStringSafe(inline, "beamType", "");
                        if (!bt.isEmpty()) {
                            if (bt.equals("|NORMAL")) inlineType = BeamContainer.BEAM_NORMAL;
                            else if (bt.equals("|SUPPORT")) inlineType = BeamContainer.BEAM_SUPPORT;
                            else if (bt.equals("|BOUNDED")) inlineType = BeamContainer.BEAM_BOUNDED;
                            else if (bt.equals("|LBEAM")) inlineType = BeamContainer.BEAM_LBEAM;
                            else if (bt.equals("|HYDRO")) inlineType = BeamContainer.BEAM_HYDRO;
                            else if (bt.equals("|ANISOTROPIC")) inlineType = BeamContainer.BEAM_ANISOTROPIC;
                        }
                        inlineId3 = getStringSafe(inline, "id3:", null);
                    }

                    String id1 = row.get(0).getAsString();
                    String id2 = row.get(1).getAsString();
                    vehicle.addBeam(inlineType, id1, id2, inlineId3,
                            inlineSpring, inlineDamp, inlineDeform, inlineStrength,
                            inlinePrecomp, inlinePrecompRange, inlinePrecompTime,
                            inlineShortBound, inlineLongBound, inlineShortBoundRange, inlineLongBoundRange,
                            inlineLimitS, inlineLimitD, inlineDampVelSplit, inlineDampFast,
                            inlineDampRebound, inlineDampReboundFast, inlineSpringExpansion, inlineDampExpansion, inlineTransitionZone
                    );
                }
            }
        }
    }

    // --- 3. Triangle Parsing ---
    public static void parseTriangles(JsonArray triangles, SoftBodyVehicle vehicle, int partId) {
        boolean isHeader = true;
        boolean currentCollision = true;
        for (JsonElement element : triangles) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                if (modifier.has("triangleType")) {
                    currentCollision = !modifier.get("triangleType").getAsString().equals("NONCOLLIDABLE");
                }
                continue;
            }
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 3) {
                    boolean inlineCollision = currentCollision;
                    if (row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                        if (inline.has("triangleType")) {
                            inlineCollision = !inline.get("triangleType").getAsString().equals("NONCOLLIDABLE");
                        }
                    }
                    vehicle.addTriangle(row.get(0).getAsString(), row.get(1).getAsString(), row.get(2).getAsString(), partId, inlineCollision);
                }
            }
        }
    }

    // --- 4. Torsionbar Parsing ---
    public static void parseTorsionbars(JsonArray torsionbars, SoftBodyVehicle vehicle) {
        boolean isHeader = true;
        double currentSpring = 0.0, currentDamp = 0.0;
        double currentDeform = PhysicsWorld.KINDA_BIG_NUMBER;
        double currentStrength = PhysicsWorld.KINDA_BIG_NUMBER;

        for (JsonElement element : torsionbars) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentSpring = getDoubleSafe(modifier, "spring", currentSpring);
                currentDamp = getDoubleSafe(modifier, "damp", currentDamp);
                currentDeform = getDoubleSafe(modifier, "deform", currentDeform);
                currentStrength = getDoubleSafe(modifier, "beamStrength", currentStrength);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 4) {
                    vehicle.addTorsionBar(row.get(0).getAsString(), row.get(1).getAsString(), row.get(2).getAsString(), row.get(3).getAsString(),
                            currentSpring, currentDamp, currentDeform, currentStrength);
                }
            }
        }
    }

    // --- 5. Rail Parsing ---
    public static void parseRails(JsonObject railsObj) {
        for (String railName : railsObj.keySet()) {
            JsonObject rail = railsObj.getAsJsonObject(railName);
            if (rail.has("links:")) {
                JsonArray links = rail.getAsJsonArray("links:");
                if (links.size() >= 2) {
                    String[] arr = new String[links.size()];
                    for (int i = 0; i < links.size(); i++) {
                        arr[i] = links.get(i).getAsString();
                    }
                    RAIL_MAP.put(railName, arr);
                }
            }
        }
    }

    // --- 6. Slidenode Parsing ---
    public static void parseSlidenodes(JsonArray slidenodes, SoftBodyVehicle vehicle) {
        boolean isHeader = true;
        for (JsonElement element : slidenodes) {
            if (element.isJsonObject()) continue;
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {
                    String nodeId = row.get(0).getAsString();
                    String railName = row.get(1).getAsString();
                    double spring = 0, damp = 0;

                    if (row.size() > 5 && !row.get(5).isJsonNull()) {
                        String sStr = row.get(5).getAsString().trim();
                        if (!sStr.isEmpty() && !sStr.contains("FLT")) {
                            try { spring = Double.parseDouble(sStr); } catch (Exception e) {}
                        }
                    }
                    String[] links = RAIL_MAP.get(railName);
                    if (links != null && links.length >= 2) {
                        vehicle.addSlideNode(nodeId, links, spring, damp);
                    }
                }
            }
        }
    }

    // --- 7. Flexbody Parsing ---
    public static void parseFlexbodies(JsonArray flexbodies, SoftBodyVehicle vehicle, int partId, JBeamAssembler.TransformContext transform) {
        boolean isHeader = true;
        for (JsonElement element : flexbodies) {
            if (element.isJsonObject()) continue;

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }

                if (row.size() >= 2) {
                    String meshName = row.get(0).getAsString();
                    java.util.List<String> targetGroups = new java.util.ArrayList<>();
                    JsonElement groupElement = row.get(1);
                    if (groupElement.isJsonArray()) {
                        for (JsonElement g : groupElement.getAsJsonArray()) targetGroups.add(g.getAsString());
                    } else {
                        targetGroups.add(groupElement.getAsString());
                    }

                    double px = 0, py = 0, pz = 0;
                    double rx = 0, ry = 0, rz = 0;
                    double sx = 1, sy = 1, sz = 1;

                    if (row.size() >= 4 && row.get(3).isJsonObject()) {
                        JsonObject trans = row.get(3).getAsJsonObject();
                        if (trans.has("pos")) {
                            JsonObject pos = trans.getAsJsonObject("pos");
                            px = getDoubleSafe(pos, "x", 0); py = getDoubleSafe(pos, "y", 0); pz = getDoubleSafe(pos, "z", 0);
                        }
                        if (trans.has("rot")) {
                            JsonObject rot = trans.getAsJsonObject("rot");
                            rx = getDoubleSafe(rot, "x", 0); ry = getDoubleSafe(rot, "y", 0); rz = getDoubleSafe(rot, "z", 0);
                        }
                        if (trans.has("scale")) {
                            JsonObject scale = trans.getAsJsonObject("scale");
                            sx = getDoubleSafe(scale, "x", 1); sy = getDoubleSafe(scale, "y", 1); sz = getDoubleSafe(scale, "z", 1);
                        }
                    }

                    // 🚀 原汁原味录入：彻底剥离欧拉角直加与提前坐标映射
                    vehicle.flexbodies.registerFlexbody(
                            meshName, targetGroups,
                            px, py, pz,
                            rx, ry, rz,
                            sx, sy, sz,
                            partId, transform
                    );
                }
            }
        }
    }
}