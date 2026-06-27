package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import me.mzy.beamcraft.BeamCraft;

import java.util.Map;

/**
 * Parses JBeam formatted JSON data into physics world elements
 * Provides safe value extraction and handles all JBeam component types
 */
public class JBeamParser {

    /**
     * 解析 BeamNG 风格的表达式，如 "$= $tirepressure_F * 550 + 10"
     * 支持 + - * / 和变量替换（变量以 $ 开头，未定义变量 → 0.0）
     */
    public static Float evaluateBeamNGExpression(String expr, Map<String, Double> variables) {
        expr = expr.trim();

        // 处理 BeamNG 的直接变量引用 (例如 "$trackoffset_F")
        if (expr.startsWith("$") && !expr.startsWith("$=")) {
            String varName = expr.substring(1); // 砍掉前面的 '$'
            if (variables != null && variables.containsKey(varName)) {
                return variables.get(varName).floatValue();
            }
            return 0.0f; // Undefined BeamNG variables default to zero.
        }

        // 如果既不是 $= 也不是纯变量，尝试按普通数字解析
        if (!expr.startsWith("$=")) {
            try { return Float.parseFloat(expr); } catch (Exception e) { return null; }
        }

        String equation = expr.substring(2);

        // 递归替换所有 $variable (最长匹配)
        boolean changed;
        do {
            changed = false;
            StringBuilder sb = new StringBuilder();
            int i = 0;
            while (i < equation.length()) {
                if (equation.charAt(i) == '$') {
                    int j = i + 1;
                    while (j < equation.length() && (Character.isLetterOrDigit(equation.charAt(j)) || equation.charAt(j) == '_')) j++;
                    String varName = equation.substring(i + 1, j);
                    float val = variables != null ? variables.getOrDefault(varName, 0.0).floatValue() : 0.0f;
                    sb.append(val);
                    i = j;
                    changed = true;
                } else {
                    sb.append(equation.charAt(i));
                    i++;
                }
            }
            equation = sb.toString();
        } while (changed);

        // 简单四则运算 (支持 + - * /，按顺序从左到右，无优先级)
        try {
            equation = equation.replaceAll("\\s+", "");
            // 先计算乘除
            while (equation.contains("*") || equation.contains("/")) {
                int idxMul = equation.indexOf('*');
                int idxDiv = equation.indexOf('/');
                int opIdx = (idxMul >= 0 && (idxDiv < 0 || idxMul < idxDiv)) ? idxMul : idxDiv;
                if (opIdx < 0) break;
                int leftStart = opIdx - 1;
                while (leftStart >= 0 && (Character.isDigit(equation.charAt(leftStart)) || equation.charAt(leftStart) == '.')) leftStart--;
                leftStart++;
                int rightEnd = opIdx + 1;
                while (rightEnd < equation.length() && (Character.isDigit(equation.charAt(rightEnd)) || equation.charAt(rightEnd) == '.')) rightEnd++;
                float left = Float.parseFloat(equation.substring(leftStart, opIdx));
                float right = Float.parseFloat(equation.substring(opIdx + 1, rightEnd));
                float res = (equation.charAt(opIdx) == '*') ? left * right : left / right;
                equation = equation.substring(0, leftStart) + res + equation.substring(rightEnd);
            }
            // 再计算加减
            while (equation.contains("+") || (equation.contains("-") && equation.lastIndexOf('-') > 0)) {
                int idxAdd = equation.indexOf('+');
                int idxSub = equation.lastIndexOf('-');
                int opIdx = (idxAdd >= 0 && (idxSub < 0 || idxAdd < idxSub)) ? idxAdd : idxSub;
                if (opIdx < 0) break;
                int leftStart = opIdx - 1;
                while (leftStart >= 0 && (Character.isDigit(equation.charAt(leftStart)) || equation.charAt(leftStart) == '.')) leftStart--;
                leftStart++;
                int rightEnd = opIdx + 1;
                while (rightEnd < equation.length() && (Character.isDigit(equation.charAt(rightEnd)) || equation.charAt(rightEnd) == '.')) rightEnd++;
                float left = Float.parseFloat(equation.substring(leftStart, opIdx));
                float right = Float.parseFloat(equation.substring(opIdx + 1, rightEnd));
                float res = (equation.charAt(opIdx) == '+') ? left + right : left - right;
                equation = equation.substring(0, leftStart) + res + equation.substring(rightEnd);
            }
            return Float.parseFloat(equation);
        } catch (Exception e) {
            return null;
        }
    }

    public static int getIntSafe(JsonObject obj, String key, int defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;

        String str = el.getAsString().trim();
        if (str.isEmpty()) return defaultValue; // 完美拦截 ""

        try {
            return Integer.parseInt(str);
        } catch (NumberFormatException e) {
            // 如果遇到小数或者乱码，退回到默认值
            return defaultValue;
        }
    }

    public static double getDoubleSafe(JsonObject obj, String key, double defaultValue, Map<String, Double> vars) {
        return getFloatSafe(obj, key, (float) defaultValue, vars);
    }

    public static float getFloatSafe(JsonObject obj, String key, float defaultValue, Map<String, Double> vars) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;

        String str = el.getAsString().trim();
        if (str.isEmpty()) return defaultValue;

        if (str.contains("FLT_MAX") || str.contains("MAX_FLT")) return Float.MAX_VALUE;
        if (str.contains("FLT_MIN") || str.contains("MIN_FLT")) return Float.MIN_VALUE;

        if (str.startsWith("$=")) {
            Float val = evaluateBeamNGExpression(str, vars);
            return val != null ? val : defaultValue;
        }

        if (str.startsWith("$")) {
            Double val = vars != null ? vars.get(str.substring(1)) : null;
            return val != null ? val.floatValue() : defaultValue;
        }

        try {
            return Float.parseFloat(str);
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
    public static void parseNodes(JsonArray nodes, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry, CouplerRegistry couplerRegistry) {
        boolean isHeader = true;

        float currentWeight = 50.0f;
        float currentFriction = 0.5f;
        float currentSlidingFriction = -1;
        boolean currentCollision = true;
        boolean currentSelfCollision = false;

        java.util.List<String> currentGroups = new java.util.ArrayList<>();

        for (JsonElement element : nodes) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentWeight = getFloatSafe(modifier, "nodeWeight", currentWeight, entry.variables);
                currentFriction = getFloatSafe(modifier, "frictionCoef", currentFriction, entry.variables);
                currentSlidingFriction = getFloatSafe(modifier, "slidingFrictionCoef", currentSlidingFriction, entry.variables);
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

                float inlineWeight = currentWeight;
                float inlineFriction = currentFriction;
                float inlineSlidingFriction = currentSlidingFriction;
                boolean inlineCollision = currentCollision;
                boolean inlineSelfCollision = currentSelfCollision;

                java.util.List<String> inlineGroups = currentGroups;

                String inlineTag = "";
                String inlineCouplerTag = "";
                float inlineStartRadius = 0.25f;
                float inlineCouplerStrength = PhysicsWorld.KINDA_BIG_NUMBER;
                boolean inlineCouplerWeld = false;
                float inlineCouplerLatchSpeed = 0.3f;
                float inlineCouplerLockRadius = 0.025f;

                if (row.get(row.size() - 1).isJsonObject()) {
                    JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                    inlineWeight = getFloatSafe(inline, "nodeWeight", inlineWeight, entry.variables);
                    inlineFriction = getFloatSafe(inline, "frictionCoef", inlineFriction, entry.variables);
                    inlineSlidingFriction = getFloatSafe(inline, "slidingFrictionCoef", inlineSlidingFriction, entry.variables);
                    inlineCollision = getBooleanSafe(inline, "collision", inlineCollision);
                    inlineSelfCollision = getBooleanSafe(inline, "selfCollision", inlineSelfCollision);

                    if (inline.has("group")) {
                        inlineGroups = parseGroups(inline.get("group"));
                    }

                    inlineTag = getStringSafe(inline, "tag", inlineTag);
                    inlineCouplerTag = getStringSafe(inline, "couplerTag", inlineCouplerTag);
                    inlineStartRadius = getFloatSafe(inline, "couplerStartRadius", inlineStartRadius, entry.variables);
                    inlineCouplerStrength = getFloatSafe(inline, "couplerStrength", inlineCouplerStrength, entry.variables);
                    if (inline.has("couplerWeld")) inlineCouplerWeld = getBooleanSafe(inline, "couplerWeld", inlineCouplerWeld);
                    else if (inline.has("couplerLock")) inlineCouplerWeld = getBooleanSafe(inline, "couplerLock", inlineCouplerWeld);
                    inlineCouplerLatchSpeed = getFloatSafe(inline, "couplerLatchSpeed", inlineCouplerLatchSpeed, entry.variables);
                    inlineCouplerLockRadius = getFloatSafe(inline, "couplerLockRadius", inlineCouplerLockRadius, entry.variables);
                }

                String id = row.get(0).getAsString();
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;

                try {
                    // 提取原始 BeamNG 空间位置
                    float rawX = row.get(1).getAsFloat();
                    float rawY = row.get(2).getAsFloat();
                    float rawZ = row.get(3).getAsFloat();

                    // 应用插槽级联变换处理逻辑 (包含对称镜像平移、欧拉角旋转、绝对位移)
                    double[] transformed = entry.transform.transformNode(rawX, rawY, rawZ);

                    // 最终引擎空间转换: flip X, swap Y and Z
                    x = (float) transformed[0];
                    y = (float) transformed[2];
                    z = (float) -transformed[1];
                } catch (Exception e) {
                    System.err.println("⚠️ Failed to parse node coordinates, skipping: " + id);
                    continue;
                }

                if (!inlineTag.isEmpty() || !inlineCouplerTag.isEmpty()) {
                    couplerRegistry.register(id, inlineTag, inlineCouplerTag, inlineStartRadius, inlineCouplerLatchSpeed, inlineCouplerStrength, inlineCouplerWeld, inlineCouplerLockRadius);
                }

                vehicle.addNode(id, x, y, z, inlineWeight, inlineFriction, inlineSlidingFriction, entry.partId, inlineCollision, inlineSelfCollision, inlineGroups);
            }
        }
    }

    // --- 2. Beam Parsing ---
    public static void parseBeams(JsonArray beams, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        boolean isHeader = true;
        int currentType = BeamContainer.BEAM_NORMAL;

        float currentPrecomp = 1.0f, currentPrecompRange = 0.0f, currentPrecompTime = 0.0f;
        float currentSpring = 9000000.0f, currentDamp = 12000.0f;
        float currentDeform = 400000.0f, currentStrength = 1000000.0f;

        float currentShortBound = 1.0f, currentLongBound = 1.0f;
        float currentShortBoundRange = -1.0f, currentLongBoundRange = -1.0f;
        float currentLimitSpring = currentSpring, currentLimitDamp = currentDamp;

        float currentDampVelSplit = -1.0f, currentDampFast = -1.0f;
        float currentDampRebound = -1.0f, currentDampReboundFast = -1.0f;

        float currentSpringExpansion = currentSpring, currentDampExpansion = currentDamp;
        float currentTransitionZone = 0.0f;

        java.util.List<String> currentBreakGroups = new java.util.ArrayList<>();
        int currentBreakGroupType = 0;

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

                currentPrecomp = getFloatSafe(modifier, "beamPrecompression", currentPrecomp, entry.variables);
                currentPrecompRange = getFloatSafe(modifier, "precompressionRange", currentPrecompRange, entry.variables);
                currentPrecompTime = getFloatSafe(modifier, "beamPrecompressionTime", currentPrecompTime, entry.variables);
                currentSpring = getFloatSafe(modifier, "beamSpring", currentSpring, entry.variables);
                currentDamp = getFloatSafe(modifier, "beamDamp", currentDamp, entry.variables);
                currentDeform = getFloatSafe(modifier, "beamDeform", currentDeform, entry.variables);
                currentStrength = getFloatSafe(modifier, "beamStrength", currentStrength, entry.variables);

                currentShortBound = getFloatSafe(modifier, "beamShortBound", currentShortBound, entry.variables);
                currentLongBound = getFloatSafe(modifier, "beamLongBound", currentLongBound, entry.variables);
                currentShortBoundRange = getFloatSafe(modifier, "shortBoundRange", currentShortBoundRange, entry.variables);
                currentLongBoundRange = getFloatSafe(modifier, "longBoundRange", currentLongBoundRange, entry.variables);
                currentLimitSpring = getFloatSafe(modifier, "beamLimitSpring", currentLimitSpring, entry.variables);
                currentLimitDamp = getFloatSafe(modifier, "beamLimitDamp", currentLimitDamp, entry.variables);

                currentDampVelSplit = getFloatSafe(modifier, "beamDampVelocitySplit", currentDampVelSplit, entry.variables);
                currentDampFast = getFloatSafe(modifier, "beamDampFast", currentDampFast, entry.variables);
                currentDampRebound = getFloatSafe(modifier, "beamDampRebound", currentDampRebound, entry.variables);
                currentDampReboundFast = getFloatSafe(modifier, "beamDampReboundFast", currentDampReboundFast, entry.variables);

                currentSpringExpansion = getFloatSafe(modifier, "springExpansion", currentSpringExpansion, entry.variables);
                currentDampExpansion = getFloatSafe(modifier, "dampExpansion", currentDampExpansion, entry.variables);
                currentTransitionZone = getFloatSafe(modifier, "transitionZone", currentTransitionZone, entry.variables);

                if (modifier.has("breakGroup")) {
                    currentBreakGroups = parseGroups(modifier.get("breakGroup"));
                    currentBreakGroupType = getIntSafe(modifier, "breakGroupType", currentBreakGroupType);
                }
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {
                    int inlineType = currentType;
                    float inlineSpring = currentSpring, inlineDamp = currentDamp;
                    float inlineDeform = currentDeform, inlineStrength = currentStrength;
                    float inlinePrecomp = currentPrecomp, inlinePrecompRange = currentPrecompRange, inlinePrecompTime = currentPrecompTime;
                    float inlineShortBound = currentShortBound, inlineLongBound = currentLongBound;
                    float inlineShortBoundRange = currentShortBoundRange, inlineLongBoundRange = currentLongBoundRange;
                    float inlineLimitS = currentLimitSpring, inlineLimitD = currentLimitDamp;
                    float inlineDampVelSplit = currentDampVelSplit, inlineDampFast = currentDampFast;
                    float inlineDampRebound = currentDampRebound, inlineDampReboundFast = currentDampReboundFast;
                    float inlineSpringExpansion = currentSpringExpansion, inlineDampExpansion = currentDampExpansion;
                    float inlineTransitionZone = currentTransitionZone;
                    String inlineId3 = null; // for L-Beams
                    java.util.List<String> inlineBreakGroups = currentBreakGroups;
                    int inlineBreakGroupType = currentBreakGroupType;

                    if (row.size() >= 3 && row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();

                        inlineSpring = getFloatSafe(inline, "beamSpring", inlineSpring, entry.variables);
                        inlineDamp = getFloatSafe(inline, "beamDamp", inlineDamp, entry.variables);
                        inlineDeform = getFloatSafe(inline, "beamDeform", inlineDeform, entry.variables);
                        inlineStrength = getFloatSafe(inline, "beamStrength", inlineStrength, entry.variables);
                        inlinePrecomp = getFloatSafe(inline, "beamPrecompression", inlinePrecomp, entry.variables);
                        inlinePrecompRange = getFloatSafe(inline, "precompressionRange", inlinePrecompRange, entry.variables);
                        inlinePrecompTime = getFloatSafe(inline, "beamPrecompressionTime", inlinePrecompTime, entry.variables);

                        inlineShortBound = getFloatSafe(inline, "beamShortBound", inlineShortBound, entry.variables);
                        inlineLongBound = getFloatSafe(inline, "beamLongBound", inlineLongBound, entry.variables);
                        inlineShortBoundRange = getFloatSafe(inline, "shortBoundRange", inlineShortBoundRange, entry.variables);
                        inlineLongBoundRange = getFloatSafe(inline, "longBoundRange", inlineLongBoundRange, entry.variables);
                        inlineLimitS = getFloatSafe(inline, "beamLimitSpring", inlineLimitS, entry.variables);
                        inlineLimitD = getFloatSafe(inline, "beamLimitDamp", inlineLimitD, entry.variables);

                        inlineDampVelSplit = getFloatSafe(inline, "beamDampVelocitySplit", inlineDampVelSplit, entry.variables);
                        inlineDampFast = getFloatSafe(inline, "beamDampFast", inlineDampFast, entry.variables);
                        inlineDampRebound = getFloatSafe(inline, "beamDampRebound", inlineDampRebound, entry.variables);
                        inlineDampReboundFast = getFloatSafe(inline, "beamDampReboundFast", inlineDampReboundFast, entry.variables);

                        inlineSpringExpansion = getFloatSafe(inline, "springExpansion", inlineSpringExpansion, entry.variables);
                        inlineDampExpansion = getFloatSafe(inline, "dampExpansion", inlineDampExpansion, entry.variables);
                        inlineTransitionZone = getFloatSafe(inline, "transitionZone", inlineTransitionZone, entry.variables);

                        if (inline.has("breakGroup")) {
                            inlineBreakGroups = parseGroups(inline.get("breakGroup"));
                            inlineBreakGroupType = getIntSafe(inline, "breakGroupType", inlineBreakGroupType);
                        }

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
                            inlineBreakGroups, inlineBreakGroupType,
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
    public static void parseTriangles(JsonArray triangles, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
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
                    vehicle.addTriangle(row.get(0).getAsString(), row.get(1).getAsString(), row.get(2).getAsString(), entry.partId, inlineCollision);
                }
            }
        }
    }

    // --- 4. Torsionbar Parsing ---
    public static void parseTorsionbars(JsonArray torsionbars, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        boolean isHeader = true;
        float currentSpring = 0.0f, currentDamp = 0.0f;
        float currentDeform = PhysicsWorld.KINDA_BIG_NUMBER;
        float currentStrength = PhysicsWorld.KINDA_BIG_NUMBER;

        for (JsonElement element : torsionbars) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentSpring = getFloatSafe(modifier, "spring", currentSpring, entry.variables);
                currentDamp = getFloatSafe(modifier, "damp", currentDamp, entry.variables);
                currentDeform = getFloatSafe(modifier, "deform", currentDeform, entry.variables);
                currentStrength = getFloatSafe(modifier, "beamStrength", currentStrength, entry.variables);
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
    public static void parseRails(JsonObject railsObj, Map<String, String[]> globalRailMap) {
        for (String railName : railsObj.keySet()) {
            JsonObject rail = railsObj.getAsJsonObject(railName);
            if (rail.has("links:")) {
                JsonArray links = rail.getAsJsonArray("links:");
                if (links.size() >= 2) {
                    String[] arr = new String[links.size()];
                    for (int i = 0; i < links.size(); i++) {
                        arr[i] = links.get(i).getAsString();
                    }
                    globalRailMap.put(railName, arr);
                }
            }
        }
    }

    // --- 6. Slidenode Parsing ---
    public static void parseSlidenodes(JsonArray slidenodes, Map<String, String[]> globalRailMap, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        boolean isHeader = true;

        // 给定默认值
        float currentSpring = 0.0f;
        float currentDamp = 0.0f;

        for (JsonElement element : slidenodes) {

            // 1. 拦截全局修饰符（字典 {}），这里就可以用 getDoubleSafe 了！
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentSpring = getFloatSafe(modifier, "spring", currentSpring, entry.variables);
                currentDamp = getFloatSafe(modifier, "damp", currentDamp, entry.variables);
                continue;
            }

            // 2. 解析数据行（数组 []）
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }

                // 确保这行至少有 id 和 railName
                if (row.size() >= 2) {
                    String nodeId = row.get(0).getAsString();
                    String railName = row.get(1).getAsString();

                    // 继承全局状态
                    float inlineSpring = currentSpring;
                    float inlineDamp = currentDamp;

                    // 3. 拦截行内修饰符（字典 {}），比如 ["fh4r", "strut_FR", {"spring": 12000}]
                    if (row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                        inlineSpring = getFloatSafe(inline, "spring", inlineSpring, entry.variables);
                        inlineDamp = getFloatSafe(inline, "damp", inlineDamp, entry.variables);
                    }
                    // 4. 兼容那个偷懒的旧写法（按格子顺序读）
                    else if (row.size() > 5) {
                        try {
                            String sStr = row.get(5).getAsString().trim();
                            // 过滤掉 FLT_MAX 这种没用的占位符
                            if (!sStr.isEmpty() && !sStr.contains("FLT")) {
                                inlineSpring = Float.parseFloat(sStr);
                            }
                        } catch (Exception ignored) {} // 如果读不到数字就算了，用默认值
                    }

                    // 绑定到轨道上
                    String[] links = globalRailMap.get(railName);
                    if (links != null && links.length >= 2) {
                        vehicle.addSlideNode(nodeId, links, inlineSpring, inlineDamp);
                    }
                }
            }
        }
    }

    // --- 7. Flexbody Parsing ---
    public static void parseFlexbodies(JsonArray flexbodies, SoftBodyVehicle vehicle, String rootPartName, JBeamAssembler.PartEntry entry) {
        boolean isHeader = true;
        java.util.List<String> currentGroups = new java.util.ArrayList<>();

        for (JsonElement element : flexbodies) {
            // 拦截并更新全局状态修改器
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                if (modifier.has("group")) {
                    currentGroups = parseGroups(modifier.get("group"));
                }
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }

                if (row.size() >= 1) {
                    String meshName = row.get(0).getAsString();
                    if (meshName.isEmpty()) continue;

                    // 默认使用当前上下文的 Group
                    java.util.List<String> targetGroups = new java.util.ArrayList<>(currentGroups);

                    if (row.size() >= 2 && !row.get(1).isJsonObject()) {
                        JsonElement groupElement = row.get(1);
                        // 无条件覆写！即使 JBeam 传入的是 ""，它也能正确解析为空列表，从而实现 BeamNG 的“清除 Group”指令。
                        targetGroups = parseGroups(groupElement);
                    }

                    float px = 0, py = 0, pz = 0;
                    float rx = 0, ry = 0, rz = 0;
                    float sx = 1, sy = 1, sz = 1;

                    // 提取行内末尾的位移/旋转/缩放字典
                    for (int i = 1; i < row.size(); i++) {
                        if (row.get(i).isJsonObject()) {
                            JsonObject trans = row.get(i).getAsJsonObject();
                            if (trans.has("pos")) {
                                JsonObject pos = trans.getAsJsonObject("pos");
                                px = getFloatSafe(pos, "x", 0, entry.variables);
                                py = getFloatSafe(pos, "y", 0, entry.variables);
                                pz = getFloatSafe(pos, "z", 0, entry.variables);
                            }
                            if (trans.has("rot")) {
                                JsonObject rot = trans.getAsJsonObject("rot");
                                rx = getFloatSafe(rot, "x", 0, entry.variables);
                                ry = getFloatSafe(rot, "y", 0, entry.variables);
                                rz = getFloatSafe(rot, "z", 0, entry.variables);
                            }
                            if (trans.has("scale")) {
                                JsonObject scale = trans.getAsJsonObject("scale");
                                sx = getFloatSafe(scale, "x", 1, entry.variables);
                                sy = getFloatSafe(scale, "y", 1, entry.variables);
                                sz = getFloatSafe(scale, "z", 1, entry.variables);
                            }
                        }
                    }

                    vehicle.flexbodies.registerFlexbody(
                            meshName, rootPartName, targetGroups,
                            px, py, pz,
                            rx, ry, rz,
                            sx, sy, sz,
                            entry.partId,
                            entry.transform
                    );
                }
            }
        }
    }
}
