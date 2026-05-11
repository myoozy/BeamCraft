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

    // ==========================================
    // Industrial-grade safe data extraction layer
    // Core utility for preventing crashes from malformed data
    // ==========================================

    /**
     * Safely extracts a double value from a JSON object
     * Handles nulls, empty strings, and BeamNG special float constants
     */
    public static double getDoubleSafe(JsonObject obj, String key, double defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;

        String str = el.getAsString().trim();
        if (str.isEmpty()) return defaultValue;

        // Translate BeamNG special float constants
        if (str.contains("FLT_MAX") || str.contains("MAX_FLT")) return PhysicsWorld.KINDA_BIG_NUMBER;
        if (str.contains("FLT_MIN") || str.contains("MIN_FLT")) return PhysicsWorld.KINDA_SMALL_NUMBER;

        try {
            return Double.parseDouble(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    /**
     * Safely extracts a trimmed string from a JSON object
     */
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

    /**
     * 兼容 BeamNG 规范的通用 Group 提取器
     * 支持读取纯字符串、字符串数组，并处理空字符串清空逻辑
     */
    public static java.util.List<String> parseGroups(JsonElement el) {
        java.util.List<String> list = new java.util.ArrayList<>();
        if (el == null || el.isJsonNull()) return list;

        if (el.isJsonPrimitive()) {
            String g = el.getAsString().trim();
            if (!g.isEmpty()) {
                list.add(g);
            }
            // 如果 g 刚好是 ""，直接返回空 list 实现清空逻辑
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
    /**
     * Parses node definitions and creates physical nodes
     * Handles coordinate system conversion and inline property modifiers
     */
    public static void parseNodes(JsonArray nodes, SoftBodyVehicle vehicle, int partId, CouplerRegistry couplerRegistry, double offX, double offY, double offZ) {
        boolean isHeader = true;

        // Default values from Rig of Rods
        double currentWeight = 50.0;
        double currentFriction = 0.5;
        double currentSlidingFriction = -1;
        boolean currentCollision = true;
        boolean currentSelfCollision = false;

        java.util.List<String> currentGroups = new java.util.ArrayList<>();

        // TODO: JBeam中关于coupler的部分貌似全部都是inline的，如果未来发现不inline的定义，再做修改
        // String currentTag = "";
        // String currentCouplerTag = "";
        // double currentStartRadius = 0.2;
        // double currentCouplerStrength = PhysicsWorld.KINDA_BIG_NUMBER;
        // boolean currentCouplerWeld = false;
        // double currentCouplerLatchSpeed = 0.2;

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

                // Apply inline properties from the end of the row
                double inlineWeight = currentWeight;
                double inlineFriction = currentFriction;
                double inlineSlidingFriction = currentSlidingFriction;
                boolean inlineCollision = currentCollision;
                boolean inlineSelfCollision = currentSelfCollision;

                java.util.List<String> inlineGroups = currentGroups;

                // coupler
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
                    // 先读取 BeamNG 的原始坐标，并加上插槽偏移量
                    double originalX = row.get(1).getAsDouble();
                    double appliedOffX = offX;
                    // 如果节点原本在车辆右侧 (BeamNG里右侧X为负)，则偏移量取反！
                    if (originalX < 0.0) {
                        appliedOffX = -offX;
                    }

                    // 加上修正后的偏移量
                    double rawX = originalX + appliedOffX;
                    double rawY = row.get(2).getAsDouble() + offY;
                    double rawZ = row.get(3).getAsDouble() + offZ;

                    // 然后再执行坐标系转换：flip X, swap Y and Z
                    x = +rawX;
                    y = +rawZ;
                    z = -rawY;
                } catch (Exception e) {
                    System.err.println("⚠️ Failed to parse node coordinates, skipping: " + id);
                    continue;
                }

                // 注入到注册表
                if (!inlineTag.isEmpty() || !inlineCouplerTag.isEmpty()) {
                    couplerRegistry.register(id, inlineTag, inlineCouplerTag, inlineStartRadius, inlineCouplerLatchSpeed, inlineCouplerStrength, inlineCouplerWeld, inlineCouplerLockRadius);
                }

                vehicle.addNode(id, x, y, z, inlineWeight, inlineFriction, inlineSlidingFriction, partId, inlineCollision, inlineSelfCollision, inlineGroups);
            }
        }
    }

    // --- 2. Beam Parsing ---
    /**
     * Parses beam definitions and creates physical connections
     * Supports multiple beam types and inline property overrides
     */
    public static void parseBeams(JsonArray beams, SoftBodyVehicle vehicle, int partId) {
        boolean isHeader = true;

        int currentType = BeamContainer.BEAM_NORMAL;

        // Default values from Rig of Rods
        double currentPrecomp = 1.0;
        double currentPrecompRange = 0.0;
        double currentPrecompTime = 0.0;
        double currentSpring = 9000000.0, currentDamp = 12000.0;
        double currentDeform = 400000.0;
        double currentStrength = 1000000.0;

        double currentShortBound = 1.0, currentLongBound = 1.0;
        double currentShortBoundRange = -1.0, currentLongBoundRange = -1.0;
        double currentLimitSpring = currentSpring, currentLimitDamp = currentDamp;

        double currentDampVelSplit = -1.0;
        double currentDampFast = -1.0;
        double currentDampRebound = -1.0;
        double currentDampReboundFast = -1.0;

        double currentSpringExpansion = currentSpring;
        double currentDampExpansion = currentDamp;
        double currentTransitionZone = 0.0;

        for (JsonElement element : beams) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();

                String bt = getStringSafe(modifier, "beamType", "");
                if (!bt.isEmpty()) {
                    if (bt.equals("|NORMAL")) {
                        currentType = BeamContainer.BEAM_NORMAL;
                    } else if (bt.equals("|SUPPORT")) {
                        currentType = BeamContainer.BEAM_SUPPORT;
                    } else if (bt.equals("|BOUNDED")) {
                        currentType = BeamContainer.BEAM_BOUNDED;
                    } else if (bt.equals("|LBEAM")) {
                        currentType = BeamContainer.BEAM_LBEAM;
                    } else if (bt.equals("|HYDRO")) {
                        currentType = BeamContainer.BEAM_HYDRO;
                    } else if (bt.equals("|ANISOTROPIC")) {
                        currentType = BeamContainer.BEAM_ANISOTROPIC;
                    }
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

                    // Copy current state to local inline variables
                    int inlineType = currentType;
                    double inlineSpring = currentSpring,                    inlineDamp = currentDamp;
                    double inlineDeform = currentDeform,                    inlineStrength = currentStrength;
                    double inlinePrecomp = currentPrecomp,                  inlinePrecompRange = currentPrecompRange;
                    double inlinePrecompTime = currentPrecompTime;
                    double inlineShortBound = currentShortBound,            inlineLongBound = currentLongBound;
                    double inlineShortBoundRange = currentShortBoundRange,  inlineLongBoundRange = currentLongBoundRange;
                    double inlineLimitS = currentLimitSpring,               inlineLimitD = currentLimitDamp;
                    double inlineDampVelSplit = currentDampVelSplit,        inlineDampFast = currentDampFast;
                    double inlineDampRebound = currentDampRebound,          inlineDampReboundFast = currentDampReboundFast;
                    double inlineSpringExpansion = currentSpringExpansion,  inlineDampExpansion = currentDampExpansion;
                    double inlineTransitionZone = currentTransitionZone;
                    String inlineId3 = null;

                    // Apply inline properties from the end of the row
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
                    String id3 = inlineId3;

                    vehicle.addBeam(inlineType, id1, id2, id3,
                            inlineSpring, inlineDamp,
                            inlineDeform, inlineStrength,
                            inlinePrecomp, inlinePrecompRange, inlinePrecompTime,
                            inlineShortBound, inlineLongBound,
                            inlineShortBoundRange, inlineLongBoundRange,
                            inlineLimitS, inlineLimitD,
                            inlineDampVelSplit, inlineDampFast,
                            inlineDampRebound, inlineDampReboundFast,
                            inlineSpringExpansion, inlineDampExpansion, inlineTransitionZone
                    );
                }
            }
        }
    }

    // --- 3. Triangle Parsing ---
    /**
     * Parses triangle collision surfaces
     */
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
    /**
     * Parses torsion bar joint definitions
     */
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
    /**
     * Parses rail definitions and stores node sequences
     */
    public static void parseRails(JsonObject railsObj) {
        for (String railName : railsObj.keySet()) {
            JsonObject rail = railsObj.getAsJsonObject(railName);
            if (rail.has("links:")) {
                JsonArray links = rail.getAsJsonArray("links:");
                if (links.size() >= 2) {
                    // Store complete rail node sequence
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
    /**
     * Parses sliding nodes that move along predefined rails
     */
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

                    double spring = 0;
                    double damp = 0;

                    if (row.size() > 5 && !row.get(5).isJsonNull()) {
                        String sStr = row.get(5).getAsString().trim();
                        if (!sStr.isEmpty() && !sStr.contains("FLT")) {
                            try {
                                spring = Double.parseDouble(sStr);
                            } catch (Exception e) {}
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

    // --- 7. Flexbody Parsing (Render Mesh Binding) ---
    /**
     * 解析 3D 网格模型与物理节点的绑定关系 (留作后续渲染使用)
     */
    public static void parseFlexbodies(JsonArray flexbodies, SoftBodyVehicle vehicle, int partId) {
        boolean isHeader = true;
        for (JsonElement element : flexbodies) {
            if (element.isJsonObject()) continue; // flexbodies 偶尔也会有 inline modifier，目前先忽略

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }

                if (row.size() >= 2) {
                    String meshName = row.get(0).getAsString();

                    // 解析它绑定的 Node Groups (可能是一个字符串，也可能是一个数组)
                    java.util.List<String> targetGroups = new java.util.ArrayList<>();
                    JsonElement groupElement = row.get(1);
                    if (groupElement.isJsonArray()) {
                        for (JsonElement g : groupElement.getAsJsonArray()) {
                            targetGroups.add(g.getAsString());
                        }
                    } else {
                        targetGroups.add(groupElement.getAsString());
                    }

                    // 提取位移、旋转、缩放属性
                    double px = 0, py = 0, pz = 0;
                    double rx = 0, ry = 0, rz = 0;
                    double sx = 1, sy = 1, sz = 1;

                    if (row.size() >= 4 && row.get(3).isJsonObject()) {
                        JsonObject transform = row.get(3).getAsJsonObject();
                        if (transform.has("pos")) {
                            JsonObject pos = transform.getAsJsonObject("pos");
                            px = getDoubleSafe(pos, "x", 0); py = getDoubleSafe(pos, "y", 0); pz = getDoubleSafe(pos, "z", 0);
                        }
                        if (transform.has("rot")) {
                            JsonObject rot = transform.getAsJsonObject("rot");
                            rx = getDoubleSafe(rot, "x", 0); ry = getDoubleSafe(rot, "y", 0); rz = getDoubleSafe(rot, "z", 0);
                        }
                        if (transform.has("scale")) {
                            JsonObject scale = transform.getAsJsonObject("scale");
                            sx = getDoubleSafe(scale, "x", 1); sy = getDoubleSafe(scale, "y", 1); sz = getDoubleSafe(scale, "z", 1);
                        }
                    }

                    vehicle.flexbodies.registerFlexbody(meshName, targetGroups, px, py, pz, rx, ry, rz, sx, sy, sz, partId);
                }
            }
        }
    }
}