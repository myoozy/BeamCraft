package me.mzy.beamcraft.physics;

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


    // --- 1. Node Parsing ---
    /**
     * Parses node definitions and creates physical nodes
     * Handles coordinate system conversion and inline property modifiers
     */
    public static void parseNodes(JsonArray nodes, SoftBodyVehicle vehicle, int partId, CouplerRegistry couplerRegistry) {
        boolean isHeader = true;

        // Default values from Rig of Rods
        double currentWeight = 50.0;
        double currentFriction = 0.5;
        double currentSlidingFriction = -1;
        boolean currentCollision = true;
        boolean currentSelfCollision = false;

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

                // coupler
                String inlineTag = "";
                String inlineCouplerTag = "";
                double inlineStartRadius = 0.2;
                double inlineCouplerStrength = PhysicsWorld.KINDA_BIG_NUMBER;
                boolean inlineCouplerWeld = false;
                double inlineCouplerLatchSpeed = 0.2;

                if (row.get(row.size() - 1).isJsonObject()) {
                    JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                    inlineWeight = getDoubleSafe(inline, "nodeWeight", inlineWeight);
                    inlineFriction = getDoubleSafe(inline, "frictionCoef", inlineFriction);
                    inlineSlidingFriction = getDoubleSafe(inline, "slidingFrictionCoef", inlineSlidingFriction);
                    inlineCollision = getBooleanSafe(inline, "collision", inlineCollision);
                    inlineSelfCollision = getBooleanSafe(inline, "selfCollision", inlineSelfCollision);

                    inlineTag = getStringSafe(inline, "tag", inlineTag);
                    inlineCouplerTag = getStringSafe(inline, "couplerTag", inlineCouplerTag);
                    inlineStartRadius = getDoubleSafe(inline, "couplerStartRadius", inlineStartRadius);
                    inlineCouplerStrength = getDoubleSafe(inline, "couplerStrength", inlineCouplerStrength);
                    if (inline.has("couplerWeld")) inlineCouplerWeld = getBooleanSafe(inline, "couplerWeld", inlineCouplerWeld);
                    else if (inline.has("couplerLock")) inlineCouplerWeld = getBooleanSafe(inline, "couplerLock", inlineCouplerWeld);
                    inlineCouplerLatchSpeed = getDoubleSafe(inline, "couplerLatchSpeed", inlineCouplerLatchSpeed);
                }

                String id = row.get(0).getAsString();
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;

                try {
                    // Coordinate system conversion: flip X, swap Y and Z
                    x = +row.get(1).getAsDouble();
                    y = +row.get(3).getAsDouble();
                    z = -row.get(2).getAsDouble();
                } catch (Exception e) {
                    System.err.println("⚠️ Failed to parse node coordinates, skipping: " + id);
                    continue;
                }

                // 注入到注册表
                if (!inlineTag.isEmpty() || !inlineCouplerTag.isEmpty()) {
                    couplerRegistry.register(id, inlineTag, inlineCouplerTag, inlineStartRadius, inlineCouplerLatchSpeed, inlineCouplerStrength, inlineCouplerWeld);
                }

                vehicle.addNode(id, x, y, z, inlineWeight, inlineFriction, inlineSlidingFriction, partId, inlineCollision, inlineSelfCollision);
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
                currentLimitSpring = getDoubleSafe(modifier, "beamLimitSpring", currentLimitSpring);
                currentLimitDamp = getDoubleSafe(modifier, "beamLimitDamp", currentLimitDamp);

                currentDampVelSplit = getDoubleSafe(modifier, "beamDampVelocitySplit", currentDampVelSplit);
                currentDampFast = getDoubleSafe(modifier, "beamDampFast", currentDampFast);
                currentDampRebound = getDoubleSafe(modifier, "beamDampRebound", currentDampRebound);
                currentDampReboundFast = getDoubleSafe(modifier, "beamDampReboundFast", currentDampReboundFast);

                if (modifier.has("beamShortBound")) currentShortBoundRange = -1.0;
                if (modifier.has("beamLongBound")) currentLongBoundRange = -1.0;

                currentShortBoundRange = getDoubleSafe(modifier, "shortBoundRange", currentShortBoundRange);
                currentLongBoundRange = getDoubleSafe(modifier, "longBoundRange", currentLongBoundRange);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {

                    // Copy current state to local inline variables
                    int inlineType = currentType;
                    double inlineSpring = currentSpring, inlineDamp = currentDamp;
                    double inlineDeform = currentDeform, inlineStrength = currentStrength;
                    double inlinePrecomp = currentPrecomp, inlinePrecompRange = currentPrecompRange;
                    double inlinePrecompTime = currentPrecompTime;
                    double inlineShortBound = currentShortBound, inlineLongBound = currentLongBound;
                    double inlineShortBoundRange = currentShortBoundRange, inlineLongBoundRange = currentLongBoundRange;
                    double inlineLimitS = currentLimitSpring, inlineLimitD = currentLimitDamp;
                    double inlineDampVelSplit = currentDampVelSplit;
                    double inlineDampFast = currentDampFast;
                    double inlineDampRebound = currentDampRebound;
                    double inlineDampReboundFast = currentDampReboundFast;

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

                        String bt = getStringSafe(inline, "beamType", "");
                        if (!bt.isEmpty()) {
                            if (bt.equals("|NORMAL")) inlineType = BeamContainer.BEAM_NORMAL;
                            else if (bt.equals("|SUPPORT")) inlineType = BeamContainer.BEAM_SUPPORT;
                            else if (bt.equals("|BOUNDED")) inlineType = BeamContainer.BEAM_BOUNDED;
                            else if (bt.equals("|LBEAM")) inlineType = BeamContainer.BEAM_LBEAM;
                            else if (bt.equals("|HYDRO")) inlineType = BeamContainer.BEAM_HYDRO;
                            else if (bt.equals("|ANISOTROPIC")) inlineType = BeamContainer.BEAM_ANISOTROPIC;
                        }
                    }

                    String id1 = row.get(0).getAsString();
                    String id2 = row.get(1).getAsString();

                    vehicle.addBeam(inlineType, id1, id2,
                            inlineSpring, inlineDamp,
                            inlineDeform, inlineStrength,
                            inlinePrecomp, inlinePrecompRange, inlinePrecompTime,
                            inlineShortBound, inlineLongBound,
                            inlineShortBoundRange, inlineLongBoundRange,
                            inlineLimitS, inlineLimitD,
                            inlineDampVelSplit, inlineDampFast,
                            inlineDampRebound, inlineDampReboundFast
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

                    // 提取位移、旋转、缩放属性 (通常在数组的第 4 个元素)
                    double px = 0, py = 0, pz = 0;
                    double rx = 0, ry = 0, rz = 0;
                    double sx = 1, sy = 1, sz = 1;

                    if (row.size() >= 4 && row.get(3).isJsonObject()) {
                        JsonObject transform = row.get(3).getAsJsonObject();
                        if (transform.has("pos")) {
                            JsonObject pos = transform.getAsJsonObject("pos");
                            px = getDoubleSafe(pos, "x", 0);
                            py = getDoubleSafe(pos, "y", 0);
                            pz = getDoubleSafe(pos, "z", 0);
                        }
                        if (transform.has("rot")) {
                            JsonObject rot = transform.getAsJsonObject("rot");
                            rx = getDoubleSafe(rot, "x", 0);
                            ry = getDoubleSafe(rot, "y", 0);
                            rz = getDoubleSafe(rot, "z", 0);
                        }
                        if (transform.has("scale")) {
                            JsonObject scale = transform.getAsJsonObject("scale");
                            sx = getDoubleSafe(scale, "x", 1);
                            sy = getDoubleSafe(scale, "y", 1);
                            sz = getDoubleSafe(scale, "z", 1);
                        }
                    }

                    // TODO: 存入 vehicle.renderEngine.addFlexbody(...)
                    // System.out.println("Registered Mesh: " + meshName + " bound to " + targetGroups);
                }
            }
        }
    }

    // --- 8. PressureWheels Parsing ---
    public static void parsePressureWheels(JsonArray pressureWheels, SoftBodyVehicle vehicle, PressureWheelsState pool) {
        if (true)return;

        boolean isHeader = true;

        for (JsonElement element : pressureWheels) {
            if (element.isJsonObject()) {
                JsonObject mod = element.getAsJsonObject();
                // 检查是否有给后续修饰符命名的键
                if (mod.has("name") && mod.get("name").isJsonPrimitive()) {
                    pool.setGroupName(mod.get("name").getAsString());
                } else if (mod.has("group") && mod.get("group").isJsonPrimitive()) {
                    pool.setGroupName(mod.get("group").getAsString());
                } else if (mod.has("hubGroup") && mod.get("hubGroup").isJsonPrimitive()) {
                    pool.setGroupName(mod.get("hubGroup").getAsString());
                }

                // 将属性存入当前组
                pool.updateCurrentState(mod);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }

                if (row.size() >= 5) {
                    String wheelName = row.get(0).getAsString();
                    String hubGroup = row.get(1).isJsonNull() ? "default" : row.get(1).getAsString(); // 从数据行获取组名
                    String tireGroup = row.get(2).isJsonNull() ? "default" : row.get(2).getAsString();

                    Integer n1 = vehicle.nodes.nameToIndex.get(row.get(3).getAsString());
                    Integer n2 = vehicle.nodes.nameToIndex.get(row.get(4).getAsString());
                    if (n1 == null || n2 == null) continue;

                    // 注意：有时候后轮可能没有 nodeArm，或者参数不够长，做好防越界和判空
                    // 🚀 正确提取 nodeS (索引 5)
                    Integer nodeS = null;
                    if (row.size() > 5 && !row.get(5).isJsonNull()) {
                        String sName = row.get(5).getAsString();
                        if (!sName.equals("9999")) { // 9999 在 BeamNG 中代表禁用此节点
                            nodeS = vehicle.nodes.nameToIndex.get(sName);
                        }
                    }

                    // 🚀 修正 nodeArm 的提取 (索引 6)
                    Integer nodeArm = null;
                    if (row.size() > 6 && !row.get(6).isJsonNull()) {
                        String armName = row.get(6).getAsString();
                        if (!armName.equals("9999")) {
                            nodeArm = vehicle.nodes.nameToIndex.get(armName);
                        }
                    }

                    // 🚀 获取该车轴所属组累积的属性 (优先取轮胎组名，如果没有取轮毂组名)
                    String targetGroup = tireGroup.equals("default") ? hubGroup : tireGroup;
                    PressureWheelsState.WheelState finalState = pool.getMergedState(targetGroup);

                    // 如果行末有行内覆盖，直接更新
                    if (row.get(row.size() - 1).isJsonObject()) {
                        finalState.updateFrom(row.get(row.size() - 1).getAsJsonObject());
                    }

                    // 提取最终下发参数
                    boolean finalHasTire = finalState.hasTire != null ? finalState.hasTire : true;
                    int finalRays = finalState.numRays != null ? finalState.numRays : 12;
                    double finalOff = finalState.wheelOffset != null ? finalState.wheelOffset : 0.0;

                    double hubR = finalState.hubRadius != null ? finalState.hubRadius : 0.5;
                    double hubW = finalState.hubWidth != null ? finalState.hubWidth : 0.2;
                    double hubMass = finalState.nodeWeight != null ? finalState.nodeWeight : 0.75; // 回退使用 nodeWeight
                    double hubFric = finalState.frictionCoef != null ? finalState.frictionCoef : 0.5;

                    double tireR = finalState.radius != null ? finalState.radius : 1;
                    double tireW = finalState.tireWidth != null ? finalState.tireWidth : 0.2;
                    double tireMass = finalState.nodeWeight != null ? finalState.nodeWeight : 0.15;
                    double tireFric = finalState.frictionCoef != null ? finalState.frictionCoef : 1.0;
                    double press = finalState.pressurePSI != null ? finalState.pressurePSI : 30.0;

                    // 兜底刚度
                    double hTS = finalState.treadS != null ? finalState.treadS : 1.5e6;
                    double hTD = finalState.treadD != null ? finalState.treadD : 15;
                    double hPS = finalState.periS != null ? finalState.periS : 1.5e6;
                    double hPD = finalState.periD != null ? finalState.periD : 15;
                    double hSS = finalState.sideS != null ? finalState.sideS : 1.5e6;
                    double hSD = finalState.sideD != null ? finalState.sideD : 15;

                    double tTS = finalState.treadS != null ? finalState.treadS : 40000;
                    double tTD = finalState.treadD != null ? finalState.treadD : 80;
                    double tPS = finalState.periS != null ? finalState.periS : 40000;
                    double tPD = finalState.periD != null ? finalState.periD : 40;
                    double tSS = finalState.sideS != null ? finalState.sideS : 15000;
                    double tSD = finalState.sideD != null ? finalState.sideD : 30;

                    double rS = finalState.reinfS != null ? finalState.reinfS : 20000;
                    double rD = finalState.reinfD != null ? finalState.reinfD : 180;

                    // 生成
                    vehicle.wheels.generateHub(wheelName, n1, n2, nodeS, nodeArm, finalRays, hubR, hubW, finalOff, hubMass, hubFric, hTS, hTD, hPS, hPD, hSS, hSD);

                    if (finalHasTire) {
                        vehicle.wheels.generateTire(wheelName, n1, n2, finalRays, tireR, tireW, finalOff, tireMass, tireFric, press, tTS, tTD, tPS, tPD, tSS, tSD, rS, rD);
                    }
                }
            }
        }
    }
}