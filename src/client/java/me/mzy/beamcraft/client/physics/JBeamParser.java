/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see
 * LICENSES/bCDDL-1.1.txt.
 *
 * Contains adaptations from BeamNG.drive lua/common/jbeam/expressionParser.lua,
 * lua/common/jbeam/variables.lua, lua/common/jbeam/loader.lua, and
 * lua/vehicle/jbeam/stage2.lua. Java adaptation and modifications contributed
 * by M1AO and BeamCraft contributors.
 */
package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalOutcome;
import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalStatus;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Parses JBeam formatted JSON data into physics world elements
 * Provides safe value extraction and handles all JBeam component types
 */
public class JBeamParser {

    private static final float DEFAULT_BEAM_SPRING = 4_300_000.0f;
    private static final float DEFAULT_BEAM_DAMP = 580.0f;
    private static final float DEFAULT_BEAM_DEFORM = 220_000.0f;
    private static final float DEFAULT_BEAM_STRENGTH = Float.MAX_VALUE;
    private static final float DEFAULT_BEAM_LIMIT_SPRING = 1.0f;
    private static final float DEFAULT_BEAM_LIMIT_DAMP = 1.0f;
    /** Unspecified beamLimitDampRebound; the container falls back to beamLimitDamp. */
    private static final float DEFAULT_BEAM_LIMIT_DAMP_REBOUND = -1.0f;
    /** Official bounded-beam transition distance in meters. */
    private static final float DEFAULT_BEAM_BOUND_ZONE = 1.0f;

    /**
     * 解析 BeamNG 风格的表达式，如 "$= $tirepressure_F * 550 + 10"。
     * 内部委托给 {@link JBeamExpressionEvaluator}（真正的 tokenizer + 优先级 parser）。
     * 兼容语义保持不变：裸 "$var"（未定义 → 0.0f）；非 $= 的非数字串 → null。
     */
    public static Float evaluateBeamNGExpression(String expr, Map<String, Double> variables) {
        if (expr == null) return null;
        expr = expr.trim();
        if (expr.isEmpty()) return null;

        // 纯数字字面量快速路径
        if (!expr.startsWith("$")) {
            try { return Float.parseFloat(expr); } catch (NumberFormatException e) { return null; }
        }

        // 兼容路径：仅当整个字符串就是裸 "$var"（无 $=、无运算符）时才走"未定义→0.0f"逻辑；
        // "$var + 1" 这类即使没有 $= 也按完整表达式求值。
        if (!expr.startsWith("$=")) {
            if (isBareVariable(expr)) {
                String varName = expr.substring(1);
                if (variables != null && variables.containsKey(varName)) {
                    return variables.get(varName).floatValue();
                }
                return 0.0f; // Undefined BeamNG variables default to zero.
            }
            // 形如 "$a+1" / "$components.foo"（dotted）的表达式交给 evaluator
            EvalOutcome out = JBeamExpressionEvaluator.evaluate(expr, JBeamExpressionEvaluator.contextOf(variables));
            if (out.status() == EvalStatus.OK && out.value() instanceof Number n) return n.floatValue();
            if (out.status() != EvalStatus.OK) ExpressionDiagnostics.warn(expr, out.reason());
            return null;
        }

        EvalOutcome out = JBeamExpressionEvaluator.evaluate(expr, JBeamExpressionEvaluator.contextOf(variables));
        if (out.status() == EvalStatus.OK && out.value() instanceof Number n) {
            return n.floatValue();
        }
        if (out.status() != EvalStatus.OK) {
            ExpressionDiagnostics.warn(expr, out.reason());
        }
        return null;
    }

    /**
     * 字符串字段安全读取：仅在值确实包含 "$=" 前缀时才求值，其余情况原样返回。
     * 求值结果为 String 时返回该字符串；nil/数值/布尔等非字符串结果视为"未设置"返回 null。
     */
    public static String getStringEvalSafe(JsonObject obj, String key, String defaultValue, Map<String, Double> vars) {
        String raw = getStringSafe(obj, key, defaultValue);
        return evalStringValue(raw, vars);
    }

    /**
     * 对可能为 "$=..." 的原始字符串求值。无 "$=" 前缀时原样返回；
     * 有前缀时返回求值得到的字符串（非字符串结果 / 求值失败 → null）。
     */
    static String evalStringValue(String raw, Map<String, Double> vars) {
        if (raw == null || !raw.startsWith("$=")) return raw;
        EvalOutcome out = JBeamExpressionEvaluator.evaluate(raw, JBeamExpressionEvaluator.contextOf(vars));
        if (out.status() == EvalStatus.OK) {
            Object v = out.value();
            if (v instanceof String s) return s;
            return null; // nil / 数值 / 布尔 在字符串上下文中不可用
        }
        ExpressionDiagnostics.warn(raw, out.reason());
        return null;
    }

    /**
     * 判断是否为裸 "$var" 引用（无 $=、无运算符），用于兼容路径：未定义 → 0.0f。
     */
    static boolean isBareVariable(String expr) {
        if (expr == null || expr.length() < 2 || expr.charAt(0) != '$') return false;
        for (int i = 1; i < expr.length(); i++) {
            char c = expr.charAt(i);
            if (!(Character.isLetterOrDigit(c) || c == '_' || c == '.')) return false;
        }
        return true;
    }

    /**
     * 解析节点坐标单元格。数值/纯数字字符串直接解析；"$=..." 交给 evaluator；
     * 无法解析时返回 null（调用方跳过该节点）。
     */
    static Double parseNodeCoordinate(JsonElement cell, Map<String, Double> vars) {
        if (cell == null || !cell.isJsonPrimitive()) return null;
        com.google.gson.JsonPrimitive p = cell.getAsJsonPrimitive();
        if (p.isNumber()) return p.getAsDouble();
        String s = p.getAsString().trim();
        if (s.isEmpty()) return null;
        if (s.startsWith("$")) {
            Float f = evaluateBeamNGExpression(s, vars);
            return f != null ? f.doubleValue() : null;
        }
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return null;
        }
    }

    public static int getIntSafe(JsonObject obj, String key, int defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;

        String str = el.getAsString().trim();
        if (str.isEmpty()) return defaultValue; // 空串直接返回默认值

        try {
            return Integer.parseInt(str);
        } catch (NumberFormatException e) {
            // 非整数（小数或非法字符）退回默认值
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

    /**
     * Reads a scoped table property. In JBeam an empty value cancels the active
     * scope modifier, so it must restore the property's schema default rather
     * than retain the value from the previous rows.
     */
    private static float getScopedBeamFloat(JsonObject obj, String key, float currentValue,
                                            float schemaDefault, Map<String, Double> vars) {
        if (obj == null || !obj.has(key)) return currentValue;
        JsonElement element = obj.get(key);
        if (element == null || element.isJsonNull()
                || (element.isJsonPrimitive() && element.getAsString().trim().isEmpty())) {
            return schemaDefault;
        }
        return getFloatSafe(obj, key, schemaDefault, vars);
    }

    /**
     * Presence companion to {@link #getScopedBeamFloat} for properties where the
     * value alone cannot express "unspecified". {@code precompressionRange} is the
     * motivating case: it <em>overrides</em> {@code beamPrecompression} whenever it
     * is authored, including an explicit {@code 0} or a negative delta, so an
     * absent key keeps the active scope while an empty/null value cancels it and
     * restores the schema default (undefined).
     */
    private static boolean getScopedBeamDefined(JsonObject obj, String key, boolean currentDefined) {
        if (obj == null || !obj.has(key)) return currentDefined;
        JsonElement element = obj.get(key);
        if (element == null || element.isJsonNull()) return false;
        return !(element.isJsonPrimitive() && element.getAsString().trim().isEmpty());
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
        if (!el.isJsonPrimitive()) return defaultValue;
        com.google.gson.JsonPrimitive primitive = el.getAsJsonPrimitive();
        if (primitive.isBoolean()) return primitive.getAsBoolean();
        if (primitive.isNumber()) return primitive.getAsDouble() != 0.0;
        String value = primitive.getAsString().trim();
        if ("1".equals(value)) return true;
        if ("0".equals(value)) return false;
        return Boolean.parseBoolean(value);
    }

    /** Shared JBeam table-cell string conversion. */
    public static String getStringCell(JsonElement cell) {
        if (cell == null || !cell.isJsonPrimitive()) return null;
        String value = cell.getAsString().trim();
        return value.isEmpty() ? null : value;
    }

    /** Shared JBeam table-cell integer conversion. */
    public static int getIntCell(JsonElement cell, int defaultValue) {
        if (cell == null || !cell.isJsonPrimitive()) return defaultValue;
        try {
            return (int) Double.parseDouble(cell.getAsString().trim());
        } catch (NumberFormatException exception) {
            return defaultValue;
        }
    }

    /** Shared numeric cell conversion, including JBeam expressions. */
    public static double getDoubleCell(JsonElement cell, double defaultValue, Map<String, Double> variables) {
        if (cell == null || !cell.isJsonPrimitive()) return defaultValue;
        com.google.gson.JsonPrimitive primitive = cell.getAsJsonPrimitive();
        if (primitive.isNumber()) return primitive.getAsDouble();
        String value = primitive.getAsString().trim();
        if (value.isEmpty()) return defaultValue;
        if (value.startsWith("$")) {
            Float evaluated = evaluateBeamNGExpression(value, variables);
            return evaluated != null ? evaluated : defaultValue;
        }
        try {
            return Double.parseDouble(value);
        } catch (NumberFormatException exception) {
            return defaultValue;
        }
    }

    public static double getFirstDoubleSafe(JsonObject object, String firstKey, String secondKey,
                                            double defaultValue, Map<String, Double> variables) {
        return getFirstDoubleSafe(object, defaultValue, variables, firstKey, secondKey);
    }

    public static double getFirstDoubleSafe(JsonObject object, double defaultValue,
                                            Map<String, Double> variables, String... keys) {
        if (object == null) return defaultValue;
        for (String key : keys) {
            if (object.has(key)) return getDoubleSafe(object, key, defaultValue, variables);
        }
        return defaultValue;
    }

    public static List<String> getStringListSafe(JsonObject object, Map<String, Double> variables, String... keys) {
        if (object == null) return List.of();
        for (String key : keys) {
            JsonElement element = object.get(key);
            if (element != null && !element.isJsonNull()) return parseGroups(element, variables);
        }
        return List.of();
    }

    public static List<Double> getDoubleListSafe(JsonObject object, String key, Map<String, Double> variables) {
        if (object == null) return List.of();
        JsonElement element = object.get(key);
        if (element == null || element.isJsonNull()) return List.of();
        List<Double> result = new ArrayList<>();
        if (element.isJsonArray()) {
            for (JsonElement cell : element.getAsJsonArray()) {
                double value = getDoubleCell(cell, Double.NaN, variables);
                if (!Double.isNaN(value)) result.add(value);
            }
        } else {
            double value = getDoubleCell(element, Double.NaN, variables);
            if (!Double.isNaN(value)) result.add(value);
        }
        return result;
    }

    public static List<Integer> getIntListSafe(JsonObject object, String key) {
        if (object == null) return List.of();
        JsonElement element = object.get(key);
        if (element == null || element.isJsonNull()) return List.of();
        List<Integer> result = new ArrayList<>();
        if (element.isJsonArray()) {
            for (JsonElement cell : element.getAsJsonArray()) {
                int value = getIntCell(cell, Integer.MIN_VALUE);
                if (value != Integer.MIN_VALUE) result.add(value);
            }
        } else {
            int value = getIntCell(element, Integer.MIN_VALUE);
            if (value != Integer.MIN_VALUE) result.add(value);
        }
        return result;
    }

    public static boolean isHeaderRow(JsonArray row, String firstColumnName) {
        return row != null && row.size() > 0
                && firstColumnName.equalsIgnoreCase(getStringCell(row.get(0)));
    }

    public static JsonObject copyJsonObject(JsonObject source) {
        return source == null ? new JsonObject() : source.deepCopy();
    }

    /** BeamNG-style recursive object merge; overlay values win. */
    public static void mergeJsonObjectsRecursive(JsonObject base, JsonObject overlay) {
        if (base == null || overlay == null) return;
        for (Map.Entry<String, JsonElement> entry : overlay.entrySet()) {
            JsonElement existing = base.get(entry.getKey());
            JsonElement value = entry.getValue();
            if (existing != null && existing.isJsonObject() && value.isJsonObject()) {
                mergeJsonObjectsRecursive(existing.getAsJsonObject(), value.getAsJsonObject());
            } else {
                base.add(entry.getKey(), value.deepCopy());
            }
        }
    }

    public static java.util.List<String> parseGroups(JsonElement el) {
        return parseGroups(el, null);
    }

    /**
     * 解析 group 字段。条目若以 "$=" 开头则先求值（支持字符串拼接等表达式），
     * 求值得到非空字符串才作为 group 加入；nil/布尔/数值结果被丢弃。
     */
    public static java.util.List<String> parseGroups(JsonElement el, Map<String, Double> vars) {
        java.util.List<String> list = new java.util.ArrayList<>();
        if (el == null || el.isJsonNull()) return list;

        if (el.isJsonPrimitive()) {
            addGroup(list, el.getAsString(), vars);
        } else if (el.isJsonArray()) {
            for (JsonElement item : el.getAsJsonArray()) {
                if (item.isJsonPrimitive()) {
                    addGroup(list, item.getAsString(), vars);
                }
            }
        }
        return list;
    }

    private static void addGroup(java.util.List<String> list, String raw, Map<String, Double> vars) {
        String g = raw.trim();
        if (g.isEmpty()) return;
        String evaluated = evalStringValue(g, vars);
        if (evaluated != null && !evaluated.isEmpty()) {
            list.add(evaluated);
        }
    }

    // --- 1. Node Parsing ---

    /**
     * Node defaults BeamNG applies when a part's node section never authors the
     * property. Taken from {@code lua/common/jbeam/loader.lua}
     * ({@code defaultNodeWeight = 25}) and {@code lua/vehicle/jbeam/stage2.lua}
     * ({@code frictionCoef or 1}); {@code slidingFriction} stays a negative
     * sentinel because BeamNG falls back to the resolved friction coefficient, not
     * to a constant.
     */
    static final float DEFAULT_NODE_WEIGHT = 25.0f;
    static final float DEFAULT_NODE_FRICTION = 1.0f;

    /** 可变的行内默认状态，随 nodes 数组中的修饰符对象 {} 逐步更新。 */
    static final class NodeRowState {
        float weight = DEFAULT_NODE_WEIGHT;
        float friction = DEFAULT_NODE_FRICTION;
        float slidingFriction = -1.0f;
        boolean collision = true;
        boolean selfCollision = false;
        java.util.List<String> groups = new java.util.ArrayList<>();
        String tag = "";
        String couplerTag = "";
        float couplerStartRadius = 0.0f;
        float couplerStrength = PhysicsWorld.KINDA_BIG_NUMBER;
        boolean couplerWeld = false;
        float couplerLatchSpeed = 0.3f;
        float couplerLockRadius = 0.025f;
    }

    public static void parseNodes(JsonArray nodes, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry, CouplerRegistry couplerRegistry) {
        boolean isHeader = true;
        NodeRowState state = new NodeRowState();

        for (JsonElement element : nodes) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                state.weight = getFloatSafe(modifier, "nodeWeight", state.weight, entry.variables);
                state.friction = getFloatSafe(modifier, "frictionCoef", state.friction, entry.variables);
                state.slidingFriction = getFloatSafe(modifier, "slidingFrictionCoef", state.slidingFriction, entry.variables);
                state.collision = getBooleanSafe(modifier, "collision", state.collision);
                state.selfCollision = getBooleanSafe(modifier, "selfCollision", state.selfCollision);

                if (modifier.has("group")) {
                    state.groups = parseGroups(modifier.get("group"), entry.variables);
                }
                state.tag = getStringEvalSafe(modifier, "tag", state.tag, entry.variables);
                state.couplerTag = getStringEvalSafe(modifier, "couplerTag", state.couplerTag, entry.variables);
                state.couplerStartRadius = getFloatSafe(
                        modifier, "couplerStartRadius", state.couplerStartRadius, entry.variables);
                state.couplerStrength = getFloatSafe(
                        modifier, "couplerStrength", state.couplerStrength, entry.variables);
                if (modifier.has("couplerWeld")) {
                    state.couplerWeld = getBooleanSafe(modifier, "couplerWeld", state.couplerWeld);
                } else if (modifier.has("couplerLock")) {
                    state.couplerWeld = getBooleanSafe(modifier, "couplerLock", state.couplerWeld);
                }
                state.couplerLatchSpeed = getFloatSafe(
                        modifier, "couplerLatchSpeed", state.couplerLatchSpeed, entry.variables);
                state.couplerLockRadius = getFloatSafe(
                        modifier, "couplerLockRadius", state.couplerLockRadius, entry.variables);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() < 4) continue;

                PhysicsSpecs.NodeSpec spec = buildNodeSpec(row, state, entry, couplerRegistry);
                if (spec != null) {
                    vehicle.addNode(spec);
                }
            }
        }
    }

    /**
     * 将单个节点数据行解析为 NodeSpec。坐标单元格可能是 "$=..." 表达式
     * （求值后得到数值，确保表达式坐标不会导致节点被跳过）；坐标无法解析时返回 null。
     */
    static PhysicsSpecs.NodeSpec buildNodeSpec(JsonArray row, NodeRowState state, JBeamAssembler.PartEntry entry, CouplerRegistry couplerRegistry) {
        float inlineWeight = state.weight;
        float inlineFriction = state.friction;
        float inlineSlidingFriction = state.slidingFriction;
        boolean inlineCollision = state.collision;
        boolean inlineSelfCollision = state.selfCollision;

        java.util.List<String> inlineGroups = state.groups;

        String inlineTag = state.tag;
        String inlineCouplerTag = state.couplerTag;
        float inlineStartRadius = state.couplerStartRadius;
        float inlineCouplerStrength = state.couplerStrength;
        boolean inlineCouplerWeld = state.couplerWeld;
        float inlineCouplerLatchSpeed = state.couplerLatchSpeed;
        float inlineCouplerLockRadius = state.couplerLockRadius;

        if (row.get(row.size() - 1).isJsonObject()) {
            JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
            inlineWeight = getFloatSafe(inline, "nodeWeight", inlineWeight, entry.variables);
            inlineFriction = getFloatSafe(inline, "frictionCoef", inlineFriction, entry.variables);
            inlineSlidingFriction = getFloatSafe(inline, "slidingFrictionCoef", inlineSlidingFriction, entry.variables);
            inlineCollision = getBooleanSafe(inline, "collision", inlineCollision);
            inlineSelfCollision = getBooleanSafe(inline, "selfCollision", inlineSelfCollision);

            if (inline.has("group")) {
                inlineGroups = parseGroups(inline.get("group"), entry.variables);
            }

            inlineTag = getStringEvalSafe(inline, "tag", inlineTag, entry.variables);
            inlineCouplerTag = getStringEvalSafe(inline, "couplerTag", inlineCouplerTag, entry.variables);
            inlineStartRadius = getFloatSafe(inline, "couplerStartRadius", inlineStartRadius, entry.variables);
            inlineCouplerStrength = getFloatSafe(inline, "couplerStrength", inlineCouplerStrength, entry.variables);
            if (inline.has("couplerWeld")) inlineCouplerWeld = getBooleanSafe(inline, "couplerWeld", inlineCouplerWeld);
            else if (inline.has("couplerLock")) inlineCouplerWeld = getBooleanSafe(inline, "couplerLock", inlineCouplerWeld);
            inlineCouplerLatchSpeed = getFloatSafe(inline, "couplerLatchSpeed", inlineCouplerLatchSpeed, entry.variables);
            inlineCouplerLockRadius = getFloatSafe(inline, "couplerLockRadius", inlineCouplerLockRadius, entry.variables);
        }

        String id = row.get(0).getAsString();

        // 提取原始 BeamNG 空间位置；$= 坐标由 evaluator 求值，未定义/非法 → null → 跳过该节点
        Double rawX = parseNodeCoordinate(row.get(1), entry.variables);
        Double rawY = parseNodeCoordinate(row.get(2), entry.variables);
        Double rawZ = parseNodeCoordinate(row.get(3), entry.variables);
        if (rawX == null || rawY == null || rawZ == null) {
            System.err.println("⚠️ Failed to parse node coordinates, skipping: " + id);
            return null;
        }

        // 应用插槽级联变换处理逻辑 (包含对称镜像平移、欧拉角旋转、绝对位移)
        double[] transformed = entry.transform.transformNode(rawX, rawY, rawZ);

        // 最终引擎空间转换: flip X, swap Y and Z
        float x = (float) transformed[0];
        float y = (float) transformed[2];
        float z = (float) -transformed[1];

        if (!inlineTag.isEmpty() || !inlineCouplerTag.isEmpty()) {
            couplerRegistry.register(id, inlineTag, inlineCouplerTag, inlineStartRadius, inlineCouplerLatchSpeed, inlineCouplerStrength, inlineCouplerWeld, inlineCouplerLockRadius);
        }

        return new PhysicsSpecs.NodeSpec(
                id, x, y, z,
                inlineWeight, inlineFriction, inlineSlidingFriction,
                entry.partId, inlineCollision, inlineSelfCollision, inlineGroups
        );
    }

    /**
     * Collects spawn-time pairs from BeamNG's modern advancedCouplerControl.
     * Runtime toggle forces and sounds are intentionally outside this first
     * compatibility step.
     */
    static void parseAdvancedCouplers(JsonObject part, Map<String, Double> variables,
                                      CouplerRegistry registry) {
        if (part == null || registry == null || !part.has("controller")
                || !part.get("controller").isJsonArray()) {
            return;
        }

        for (JsonElement element : part.getAsJsonArray("controller")) {
            if (!element.isJsonArray()) continue;
            JsonArray row = element.getAsJsonArray();
            if (row.size() < 2 || !"advancedCouplerControl".equals(getStringCell(row.get(0)))) continue;

            JsonObject options = null;
            for (int i = 1; i < row.size(); i++) {
                if (row.get(i).isJsonObject()) options = row.get(i).getAsJsonObject();
            }
            String controllerName = getStringEvalSafe(options, "name", null, variables);
            if (controllerName == null || !part.has(controllerName)
                    || !part.get(controllerName).isJsonObject()) continue;

            JsonElement couplerNodes = part.getAsJsonObject(controllerName).get("couplerNodes");
            if (couplerNodes == null || !couplerNodes.isJsonArray()) continue;
            parseAdvancedCouplerRows(
                    controllerName, couplerNodes.getAsJsonArray(), variables, registry);
        }
    }

    private static void parseAdvancedCouplerRows(String controllerName, JsonArray rows,
                                                 Map<String, Double> variables,
                                                 CouplerRegistry registry) {
        Map<String, Integer> columns = new java.util.HashMap<>();
        boolean headerSeen = false;
        for (JsonElement element : rows) {
            if (!element.isJsonArray()) continue;
            JsonArray row = element.getAsJsonArray();
            if (!headerSeen) {
                for (int i = 0; i < row.size(); i++) {
                    String name = getStringCell(row.get(i));
                    if (name != null) columns.put(name.toLowerCase(java.util.Locale.ROOT), i);
                }
                headerSeen = true;
                continue;
            }

            String node1 = advancedCellString(row, columns, "cid1");
            String node2 = advancedCellString(row, columns, "cid2");
            double strength = advancedCellDouble(
                    row, columns, "autocouplingstrength", PhysicsWorld.KINDA_BIG_NUMBER, variables);
            double lockRadius = advancedCellDouble(
                    row, columns, "autocouplinglockradius", 0.025, variables);
            double speed = advancedCellDouble(
                    row, columns, "autocouplingspeed", 0.3, variables);
            double startRadius = advancedCellDouble(
                    row, columns, "couplingstartradius", 0.0, variables);
            String breakGroup = advancedCellString(row, columns, "breakgroup");
            registry.registerDirect(controllerName, node1, node2, startRadius,
                    speed, strength, lockRadius, breakGroup);
        }
    }

    private static String advancedCellString(JsonArray row, Map<String, Integer> columns, String name) {
        Integer index = columns.get(name);
        return index == null || index < 0 || index >= row.size() ? null : getStringCell(row.get(index));
    }

    private static double advancedCellDouble(JsonArray row, Map<String, Integer> columns, String name,
                                             double defaultValue, Map<String, Double> variables) {
        Integer index = columns.get(name);
        return index == null || index < 0 || index >= row.size()
                ? defaultValue : getDoubleCell(row.get(index), defaultValue, variables);
    }

    // --- 2. Beam Parsing ---
    public static void parseBeams(JsonArray beams, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        parseBeamRows(beams, vehicle, entry, false);
    }

    public static void parseHydros(JsonArray hydros, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        parseBeamRows(hydros, vehicle, entry, true);
    }

    private static void parseBeamRows(JsonArray beams, SoftBodyVehicle vehicle,
                                      JBeamAssembler.PartEntry entry, boolean hydroSection) {
        boolean isHeader = true;
        int currentType = hydroSection ? BeamContainer.BEAM_HYDRO : BeamContainer.BEAM_NORMAL;
        JsonObject currentProperties = new JsonObject();

        float currentPrecomp = 1.0f, currentPrecompRange = 0.0f, currentPrecompTime = 0.0f;
        // precompressionRange is only meaningful when authored: absent => use the
        // beamPrecompression multiplier, authored (even 0 or negative) => override it.
        boolean currentPrecompRangeDefined = false;
        // Official BeamNG normal/bounded beam defaults.
        float currentSpring = DEFAULT_BEAM_SPRING, currentDamp = DEFAULT_BEAM_DAMP;
        float currentDeform = DEFAULT_BEAM_DEFORM, currentStrength = DEFAULT_BEAM_STRENGTH;
        float currentDeformLimitStress = Float.MAX_VALUE;
        // dampCutoffHz <= 0 disables the filter and keeps raw relative-velocity damping.
        float currentDampCutoffHz = -1.0f;

        float currentShortBound = 1.0f, currentLongBound = 1.0f;
        float currentShortBoundRange = -1.0f, currentLongBoundRange = -1.0f;
        float currentBoundZone = DEFAULT_BEAM_BOUND_ZONE;
        // beamLimitSpring/beamLimitDamp are independent defaults of 1, never aliases of
        // the scoped spring/damp values.
        float currentLimitSpring = DEFAULT_BEAM_LIMIT_SPRING, currentLimitDamp = DEFAULT_BEAM_LIMIT_DAMP;
        float currentLimitDampRebound = DEFAULT_BEAM_LIMIT_DAMP_REBOUND;

        // Negative sentinels mean "unspecified"; the container substitutes the fallback.
        float currentDampVelSplit = -1.0f, currentDampFast = -1.0f;
        float currentDampRebound = -1.0f, currentDampReboundFast = -1.0f;
        // beamDampVelocitySplitRebound falls back to beamDampVelocitySplit when unauthored.
        float currentDampVelSplitRebound = -1.0f;

        float currentSpringExpansion = currentSpring, currentDampExpansion = currentDamp;
        float currentTransitionZone = 0.0f;

        // Optional beam name; actuators (adaptive dampers) address beams by it.
        String currentName = null;

        java.util.List<String> currentBreakGroups = new java.util.ArrayList<>();
        java.util.List<String> currentDeformGroups = new java.util.ArrayList<>();
        float currentDeformationTriggerRatio = Float.POSITIVE_INFINITY;
        int currentBreakGroupType = 0;
        boolean currentDisableTriangleBreaking = false;

        for (JsonElement element : beams) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                if (hydroSection) {
                    mergeJsonObjectsRecursive(currentProperties, modifier);
                }
                String bt = getStringSafe(modifier, "beamType", "");
                if (!hydroSection && modifier.has("beamType")) {
                    if (bt.isEmpty() || bt.equals("|NORMAL")) currentType = BeamContainer.BEAM_NORMAL;
                    else if (bt.equals("|SUPPORT")) currentType = BeamContainer.BEAM_SUPPORT;
                    else if (bt.equals("|BOUNDED")) currentType = BeamContainer.BEAM_BOUNDED;
                    else if (bt.equals("|LBEAM")) currentType = BeamContainer.BEAM_LBEAM;
                    else if (bt.equals("|HYDRO")) currentType = BeamContainer.BEAM_HYDRO;
                    else if (bt.equals("|ANISOTROPIC")) currentType = BeamContainer.BEAM_ANISOTROPIC;
                }

                currentPrecomp = getScopedBeamFloat(modifier, "beamPrecompression", currentPrecomp, 1.0f, entry.variables);
                currentPrecompRange = getScopedBeamFloat(modifier, "precompressionRange", currentPrecompRange, 0.0f, entry.variables);
                currentPrecompRangeDefined = getScopedBeamDefined(modifier, "precompressionRange", currentPrecompRangeDefined);
                currentPrecompTime = getScopedBeamFloat(modifier, "beamPrecompressionTime", currentPrecompTime, 0.0f, entry.variables);
                currentSpring = getScopedBeamFloat(modifier, "beamSpring", currentSpring, DEFAULT_BEAM_SPRING, entry.variables);
                currentDamp = getScopedBeamFloat(modifier, "beamDamp", currentDamp, DEFAULT_BEAM_DAMP, entry.variables);
                currentDampCutoffHz = getScopedBeamFloat(modifier, "dampCutoffHz", currentDampCutoffHz, -1.0f, entry.variables);
                currentDeform = getScopedBeamFloat(modifier, "beamDeform", currentDeform, DEFAULT_BEAM_DEFORM, entry.variables);
                currentStrength = getScopedBeamFloat(modifier, "beamStrength", currentStrength, DEFAULT_BEAM_STRENGTH, entry.variables);
                currentDeformLimitStress = getScopedBeamFloat(modifier, "deformLimitStress", currentDeformLimitStress, Float.MAX_VALUE, entry.variables);

                currentShortBound = getScopedBeamFloat(modifier, "beamShortBound", currentShortBound, 1.0f, entry.variables);
                currentLongBound = getScopedBeamFloat(modifier, "beamLongBound", currentLongBound, 1.0f, entry.variables);
                currentShortBoundRange = getScopedBeamFloat(modifier, "shortBoundRange", currentShortBoundRange, -1.0f, entry.variables);
                currentLongBoundRange = getScopedBeamFloat(modifier, "longBoundRange", currentLongBoundRange, -1.0f, entry.variables);
                currentBoundZone = getScopedBeamFloat(modifier, "boundZone", currentBoundZone, DEFAULT_BEAM_BOUND_ZONE, entry.variables);
                currentLimitSpring = getScopedBeamFloat(modifier, "beamLimitSpring", currentLimitSpring, DEFAULT_BEAM_LIMIT_SPRING, entry.variables);
                currentLimitDamp = getScopedBeamFloat(modifier, "beamLimitDamp", currentLimitDamp, DEFAULT_BEAM_LIMIT_DAMP, entry.variables);
                currentLimitDampRebound = getScopedBeamFloat(modifier, "beamLimitDampRebound", currentLimitDampRebound, DEFAULT_BEAM_LIMIT_DAMP_REBOUND, entry.variables);

                currentDampVelSplit = getScopedBeamFloat(modifier, "beamDampVelocitySplit", currentDampVelSplit, -1.0f, entry.variables);
                currentDampVelSplitRebound = getScopedBeamFloat(modifier, "beamDampVelocitySplitRebound", currentDampVelSplitRebound, -1.0f, entry.variables);
                currentDampFast = getScopedBeamFloat(modifier, "beamDampFast", currentDampFast, -1.0f, entry.variables);
                currentDampRebound = getScopedBeamFloat(modifier, "beamDampRebound", currentDampRebound, -1.0f, entry.variables);
                currentDampReboundFast = getScopedBeamFloat(modifier, "beamDampReboundFast", currentDampReboundFast, -1.0f, entry.variables);

                currentSpringExpansion = getFloatSafe(modifier, "springExpansion", currentSpringExpansion, entry.variables);
                currentDampExpansion = getFloatSafe(modifier, "dampExpansion", currentDampExpansion, entry.variables);
                currentTransitionZone = getFloatSafe(modifier, "transitionZone", currentTransitionZone, entry.variables);
                currentName = getStringEvalSafe(modifier, "name", currentName, entry.variables);

                if (modifier.has("breakGroup")) {
                    currentBreakGroups = parseGroups(modifier.get("breakGroup"), entry.variables);
                    currentBreakGroupType = getIntSafe(modifier, "breakGroupType", currentBreakGroupType);
                }
                if (modifier.has("deformGroup")) {
                    currentDeformGroups = parseGroups(modifier.get("deformGroup"), entry.variables);
                }
                currentDeformationTriggerRatio = getFloatSafe(
                        modifier, "deformationTriggerRatio", currentDeformationTriggerRatio, entry.variables);
                currentDisableTriangleBreaking = getBooleanSafe(
                        modifier, "disableTriangleBreaking", currentDisableTriangleBreaking);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {
                    JsonObject rowProperties = hydroSection ? copyJsonObject(currentProperties) : null;
                    int inlineType = currentType;
                    float inlineSpring = currentSpring, inlineDamp = currentDamp;
                    float inlineDeform = currentDeform, inlineStrength = currentStrength;
                    float inlineDeformLimitStress = currentDeformLimitStress;
                    float inlinePrecomp = currentPrecomp, inlinePrecompRange = currentPrecompRange, inlinePrecompTime = currentPrecompTime;
                    boolean inlinePrecompRangeDefined = currentPrecompRangeDefined;
                    float inlineDampCutoffHz = currentDampCutoffHz;
                    float inlineShortBound = currentShortBound, inlineLongBound = currentLongBound;
                    float inlineShortBoundRange = currentShortBoundRange, inlineLongBoundRange = currentLongBoundRange;
                    float inlineBoundZone = currentBoundZone;
                    float inlineLimitS = currentLimitSpring, inlineLimitD = currentLimitDamp;
                    float inlineLimitDRebound = currentLimitDampRebound;
                    float inlineDampVelSplit = currentDampVelSplit, inlineDampFast = currentDampFast;
                    float inlineDampVelSplitRebound = currentDampVelSplitRebound;
                    float inlineDampRebound = currentDampRebound, inlineDampReboundFast = currentDampReboundFast;
                    float inlineSpringExpansion = currentSpringExpansion, inlineDampExpansion = currentDampExpansion;
                    float inlineTransitionZone = currentTransitionZone;
                    String inlineName = currentName;
                    String inlineId3 = null; // for L-Beams
                    java.util.List<String> inlineBreakGroups = currentBreakGroups;
                    java.util.List<String> inlineDeformGroups = currentDeformGroups;
                    float inlineDeformationTriggerRatio = currentDeformationTriggerRatio;
                    int inlineBreakGroupType = currentBreakGroupType;
                    boolean inlineDisableTriangleBreaking = currentDisableTriangleBreaking;

                    if (row.size() >= 3 && row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                        if (hydroSection) {
                            mergeJsonObjectsRecursive(rowProperties, inline);
                        }

                        inlineSpring = getScopedBeamFloat(inline, "beamSpring", inlineSpring, DEFAULT_BEAM_SPRING, entry.variables);
                        inlineDamp = getScopedBeamFloat(inline, "beamDamp", inlineDamp, DEFAULT_BEAM_DAMP, entry.variables);
                        inlineDampCutoffHz = getScopedBeamFloat(inline, "dampCutoffHz", inlineDampCutoffHz, -1.0f, entry.variables);
                        inlineDeform = getScopedBeamFloat(inline, "beamDeform", inlineDeform, DEFAULT_BEAM_DEFORM, entry.variables);
                        inlineStrength = getScopedBeamFloat(inline, "beamStrength", inlineStrength, DEFAULT_BEAM_STRENGTH, entry.variables);
                        inlineDeformLimitStress = getScopedBeamFloat(inline, "deformLimitStress", inlineDeformLimitStress, Float.MAX_VALUE, entry.variables);
                        inlinePrecomp = getScopedBeamFloat(inline, "beamPrecompression", inlinePrecomp, 1.0f, entry.variables);
                        inlinePrecompRange = getScopedBeamFloat(inline, "precompressionRange", inlinePrecompRange, 0.0f, entry.variables);
                        inlinePrecompRangeDefined = getScopedBeamDefined(inline, "precompressionRange", inlinePrecompRangeDefined);
                        inlinePrecompTime = getScopedBeamFloat(inline, "beamPrecompressionTime", inlinePrecompTime, 0.0f, entry.variables);

                        inlineShortBound = getScopedBeamFloat(inline, "beamShortBound", inlineShortBound, 1.0f, entry.variables);
                        inlineLongBound = getScopedBeamFloat(inline, "beamLongBound", inlineLongBound, 1.0f, entry.variables);
                        inlineShortBoundRange = getScopedBeamFloat(inline, "shortBoundRange", inlineShortBoundRange, -1.0f, entry.variables);
                        inlineLongBoundRange = getScopedBeamFloat(inline, "longBoundRange", inlineLongBoundRange, -1.0f, entry.variables);
                        inlineBoundZone = getScopedBeamFloat(inline, "boundZone", inlineBoundZone, DEFAULT_BEAM_BOUND_ZONE, entry.variables);
                        inlineLimitS = getScopedBeamFloat(inline, "beamLimitSpring", inlineLimitS, DEFAULT_BEAM_LIMIT_SPRING, entry.variables);
                        inlineLimitD = getScopedBeamFloat(inline, "beamLimitDamp", inlineLimitD, DEFAULT_BEAM_LIMIT_DAMP, entry.variables);
                        inlineLimitDRebound = getScopedBeamFloat(inline, "beamLimitDampRebound", inlineLimitDRebound, DEFAULT_BEAM_LIMIT_DAMP_REBOUND, entry.variables);

                        inlineDampVelSplit = getScopedBeamFloat(inline, "beamDampVelocitySplit", inlineDampVelSplit, -1.0f, entry.variables);
                        inlineDampVelSplitRebound = getScopedBeamFloat(inline, "beamDampVelocitySplitRebound", inlineDampVelSplitRebound, -1.0f, entry.variables);
                        inlineDampFast = getScopedBeamFloat(inline, "beamDampFast", inlineDampFast, -1.0f, entry.variables);
                        inlineDampRebound = getScopedBeamFloat(inline, "beamDampRebound", inlineDampRebound, -1.0f, entry.variables);
                        inlineDampReboundFast = getScopedBeamFloat(inline, "beamDampReboundFast", inlineDampReboundFast, -1.0f, entry.variables);

                        inlineSpringExpansion = getFloatSafe(inline, "springExpansion", inlineSpringExpansion, entry.variables);
                        inlineDampExpansion = getFloatSafe(inline, "dampExpansion", inlineDampExpansion, entry.variables);
                        inlineTransitionZone = getFloatSafe(inline, "transitionZone", inlineTransitionZone, entry.variables);

                        if (inline.has("breakGroup")) {
                            inlineBreakGroups = parseGroups(inline.get("breakGroup"), entry.variables);
                            inlineBreakGroupType = getIntSafe(inline, "breakGroupType", inlineBreakGroupType);
                        }
                        if (inline.has("deformGroup")) {
                            inlineDeformGroups = parseGroups(inline.get("deformGroup"), entry.variables);
                        }
                        inlineDeformationTriggerRatio = getFloatSafe(
                                inline, "deformationTriggerRatio", inlineDeformationTriggerRatio, entry.variables);
                        inlineDisableTriangleBreaking = getBooleanSafe(
                                inline, "disableTriangleBreaking", inlineDisableTriangleBreaking);

                        String bt = getStringSafe(inline, "beamType", "");
                        if (!hydroSection && inline.has("beamType")) {
                            if (bt.isEmpty() || bt.equals("|NORMAL")) inlineType = BeamContainer.BEAM_NORMAL;
                            else if (bt.equals("|SUPPORT")) inlineType = BeamContainer.BEAM_SUPPORT;
                            else if (bt.equals("|BOUNDED")) inlineType = BeamContainer.BEAM_BOUNDED;
                            else if (bt.equals("|LBEAM")) inlineType = BeamContainer.BEAM_LBEAM;
                            else if (bt.equals("|HYDRO")) inlineType = BeamContainer.BEAM_HYDRO;
                            else if (bt.equals("|ANISOTROPIC")) inlineType = BeamContainer.BEAM_ANISOTROPIC;
                        }
                        inlineName = getStringEvalSafe(inline, "name", inlineName, entry.variables);
                        inlineId3 = getStringSafe(inline, "id3:", null);
                    }

                    String id1 = row.get(0).getAsString();
                    String id2 = row.get(1).getAsString();
                    PhysicsSpecs.BeamSpec beamSpec = new PhysicsSpecs.BeamSpec(
                            inlineType, id1, id2, inlineId3,
                            inlineDeformGroups, inlineDeformationTriggerRatio,
                            inlineBreakGroups, inlineBreakGroupType, inlineDisableTriangleBreaking,
                            inlineSpring, inlineDamp, inlineDampCutoffHz, inlineDeform, inlineStrength,
                            inlinePrecomp, inlinePrecompRange, inlinePrecompRangeDefined, inlinePrecompTime,
                            inlineShortBound, inlineLongBound, inlineShortBoundRange, inlineLongBoundRange,
                            inlineBoundZone,
                            inlineLimitS, inlineLimitD, inlineLimitDRebound, inlineDampVelSplit, inlineDampVelSplitRebound, inlineDampFast,
                            inlineDampRebound, inlineDampReboundFast, inlineSpringExpansion, inlineDampExpansion, inlineTransitionZone,
                            inlineDeformLimitStress, inlineName
                    );
                    if (hydroSection) {
                        vehicle.addHydro(buildHydroSpec(beamSpec, rowProperties, entry.variables));
                    } else {
                        vehicle.addBeam(beamSpec);
                    }
                }
            }
        }
    }

    static PhysicsSpecs.HydroSpec buildHydroSpec(PhysicsSpecs.BeamSpec beam, JsonObject properties,
                                                  Map<String, Double> variables) {
        HydroControlValues control = buildHydroControl(properties, variables, 1.0f, 1.0f, 2.0f);

        return new PhysicsSpecs.HydroSpec(
                beam,
                control.inputSource, control.inLimit, control.outLimit, control.inputFactor,
                control.inputCenter, control.inputInLimit, control.inputOutLimit,
                control.inRate, control.outRate, control.autoCenterRate, control.steeringWheelLock
        );
    }

    // --- 3. Triangle Parsing ---
    public static void parseTriangles(JsonArray triangles, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        boolean isHeader = true;
        boolean currentCollision = true;
        java.util.List<String> currentBreakGroups = new java.util.ArrayList<>();
        for (JsonElement element : triangles) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                if (modifier.has("triangleType")) {
                    currentCollision = !modifier.get("triangleType").getAsString().equals("NONCOLLIDABLE");
                }
                if (modifier.has("breakGroup")) {
                    currentBreakGroups = parseGroups(modifier.get("breakGroup"), entry.variables);
                }
                continue;
            }
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 3) {
                    boolean inlineCollision = currentCollision;
                    java.util.List<String> inlineBreakGroups = currentBreakGroups;
                    if (row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                        if (inline.has("triangleType")) {
                            inlineCollision = !inline.get("triangleType").getAsString().equals("NONCOLLIDABLE");
                        }
                        if (inline.has("breakGroup")) {
                            inlineBreakGroups = parseGroups(inline.get("breakGroup"), entry.variables);
                        }
                    }
                    vehicle.addTriangle(new PhysicsSpecs.TriangleSpec(
                            row.get(0).getAsString(),
                            row.get(1).getAsString(),
                            row.get(2).getAsString(),
                            inlineBreakGroups,
                            entry.partId,
                            inlineCollision
                    ));
                }
            }
        }
    }

    // --- 4. Torsionbar Parsing ---
    public static void parseTorsionbars(JsonArray torsionbars, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry) {
        parseTorsionRows(torsionbars, vehicle, entry, false);
    }

    public static void parseTorsionHydros(JsonArray torsionHydros, SoftBodyVehicle vehicle,
                                           JBeamAssembler.PartEntry entry) {
        parseTorsionRows(torsionHydros, vehicle, entry, true);
    }

    private static void parseTorsionRows(JsonArray rows, SoftBodyVehicle vehicle,
                                         JBeamAssembler.PartEntry entry, boolean hydroSection) {
        boolean isHeader = true;
        JsonObject currentProperties = new JsonObject();

        for (JsonElement element : rows) {
            if (element.isJsonObject()) {
                mergeJsonObjectsRecursive(currentProperties, element.getAsJsonObject());
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 4) {
                    JsonObject properties = copyJsonObject(currentProperties);
                    if (row.get(row.size() - 1).isJsonObject()) {
                        mergeJsonObjectsRecursive(properties, row.get(row.size() - 1).getAsJsonObject());
                    }
                    float spring = getFloatSafe(properties, "spring", 0.0f, entry.variables);
                    float damp = getFloatSafe(properties, "damp", 0.0f, entry.variables);
                    PhysicsSpecs.TorsionBarSpec torsionBar = new PhysicsSpecs.TorsionBarSpec(
                            row.get(0).getAsString(),
                            row.get(1).getAsString(),
                            row.get(2).getAsString(),
                            row.get(3).getAsString(),
                            spring,
                            damp,
                            getFloatSafe(properties, "spring2", spring, entry.variables),
                            getFloatSafe(properties, "damp2", damp, entry.variables),
                            getFloatSafe(properties, "deform", PhysicsWorld.KINDA_BIG_NUMBER, entry.variables),
                            getFloatSafe(properties, "strength", PhysicsWorld.KINDA_BIG_NUMBER, entry.variables),
                            getFloatSafe(properties, "precompressionAngle", 0.0f, entry.variables),
                            Math.max(0.0f, getFloatSafe(
                                    properties, "precompressionTime", 0.0f, entry.variables))
                    );
                    if (hydroSection) {
                        HydroControlValues control = buildHydroControl(
                                properties, entry.variables, 0.0f, -1.0f, 1.0f);
                        vehicle.addTorsionHydro(new PhysicsSpecs.TorsionHydroSpec(
                                torsionBar,
                                control.inputSource, control.inLimit, control.outLimit, control.inputFactor,
                                control.inputCenter, control.inputInLimit, control.inputOutLimit,
                                control.inRate, control.outRate, control.autoCenterRate,
                                control.steeringWheelLock
                        ));
                    } else {
                        vehicle.addTorsionBar(torsionBar);
                    }
                }
            }
        }
    }

    private static HydroControlValues buildHydroControl(JsonObject properties, Map<String, Double> variables,
                                                         float neutralOutput, float defaultInLimit,
                                                         float defaultOutLimit) {
        float inLimit = getFloatSafe(properties, "inLimit", defaultInLimit, variables);
        float outLimit = getFloatSafe(properties, "outLimit", defaultOutLimit, variables);
        float inputFactor = getFloatSafe(properties, "inputFactor", 1.0f, variables);

        if (properties.has("factor") && !properties.get("factor").isJsonNull()) {
            float factor = getFloatSafe(properties, "factor", Float.NaN, variables);
            if (Float.isFinite(factor)) {
                float extent = Math.abs(factor);
                inLimit = neutralOutput - extent;
                outLimit = neutralOutput + extent;
                inputFactor = factor < 0.0f ? -1.0f : 1.0f;
            }
        }

        float inRate = Math.max(0.0f, getFloatSafe(properties, "inRate", 2.0f, variables));
        float outRate = Math.max(0.0f, getFloatSafe(properties, "outRate", inRate, variables));
        float autoCenterRate = Math.max(0.0f,
                getFloatSafe(properties, "autoCenterRate", inRate, variables));
        Float steeringWheelLock = properties.has("steeringWheelLock")
                && !properties.get("steeringWheelLock").isJsonNull()
                ? getFloatSafe(properties, "steeringWheelLock", 0.0f, variables)
                : null;
        return new HydroControlValues(
                getStringSafe(properties, "inputSource", "steering_input"),
                inLimit, outLimit, inputFactor,
                getFloatSafe(properties, "inputCenter", 0.0f, variables),
                getFloatSafe(properties, "inputInLimit", -1.0f, variables),
                getFloatSafe(properties, "inputOutLimit", 1.0f, variables),
                inRate, outRate, autoCenterRate, steeringWheelLock
        );
    }

    private record HydroControlValues(
            String inputSource, float inLimit, float outLimit, float inputFactor,
            float inputCenter, float inputInLimit, float inputOutLimit,
            float inRate, float outRate, float autoCenterRate, Float steeringWheelLock
    ) {}

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

        // BeamNG does not default a slidenode spring to zero: processSlidenodes uses
        // `snode.spring or vehicle.options.beamSpring`, i.e. an unauthored spring is
        // the global beam spring, so the node rides the rail stiffly rather than
        // floating on it. Damping has no BeamNG counterpart at all — addSlidenode
        // takes only a spring — so zero stays the right default there.
        float currentSpring = DEFAULT_BEAM_SPRING;
        float currentDamp = 0.0f;

        for (JsonElement element : slidenodes) {

            // 1. 全局修饰符（字典 {}）
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

                    // 3. 行内修饰符（字典 {}），如 ["fh4r", "strut_FR", {"spring": 12000}]
                    if (row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                        inlineSpring = getFloatSafe(inline, "spring", inlineSpring, entry.variables);
                        inlineDamp = getFloatSafe(inline, "damp", inlineDamp, entry.variables);
                    }
                    // 4. 旧写法：按格子顺序读取
                    else if (row.size() > 5) {
                        try {
                            String sStr = row.get(5).getAsString().trim();
                            // 跳过 FLT_MAX 占位符
                            if (!sStr.isEmpty() && !sStr.contains("FLT")) {
                                inlineSpring = Float.parseFloat(sStr);
                            }
                        } catch (Exception ignored) {} // 读到非数字时保留默认值
                    }

                    // 绑定到轨道
                    String[] links = globalRailMap.get(railName);
                    if (links != null && links.length >= 2) {
                        vehicle.addSlideNode(new PhysicsSpecs.SlideNodeSpec(nodeId, links, inlineSpring, inlineDamp));
                    }
                }
            }
        }
    }

    // --- 7. Flexbody Parsing ---
    public static void parseFlexbodies(JsonArray flexbodies, SoftBodyVehicle vehicle, String rootPartName, JBeamAssembler.PartEntry entry) {
        boolean isHeader = true;
        java.util.List<String> currentGroups = new java.util.ArrayList<>();
        String currentDeformGroup = "";
        String currentDeformMaterialBase = "";
        String currentDeformMaterialDamaged = "";

        for (JsonElement element : flexbodies) {
            // 更新全局状态修改器
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                if (modifier.has("group")) {
                    currentGroups = parseGroups(modifier.get("group"), entry.variables);
                }
                currentDeformGroup = getStringSafe(modifier, "deformGroup", currentDeformGroup);
                currentDeformMaterialBase = getStringSafe(
                        modifier, "deformMaterialBase", currentDeformMaterialBase);
                currentDeformMaterialDamaged = getStringSafe(
                        modifier, "deformMaterialDamaged", currentDeformMaterialDamaged);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }

                if (row.size() >= 1) {
                    String meshName = row.get(0).getAsString();
                    if (meshName.isEmpty()) continue;

                    // meshName 可能是一条 "$=..." 字符串表达式（如 $components 条件选 mesh）
                    String evaluatedMesh = evalStringValue(meshName, entry.variables);
                    if (evaluatedMesh == null || evaluatedMesh.isEmpty()) continue;
                    meshName = evaluatedMesh;

                    // 默认使用当前上下文的 Group
                    java.util.List<String> targetGroups = new java.util.ArrayList<>(currentGroups);

                    if (row.size() >= 2 && !row.get(1).isJsonObject()) {
                        JsonElement groupElement = row.get(1);
                        // 无条件覆写：即便 JBeam 传入 ""，也会解析为空列表，对应 BeamNG 的清除 Group 指令。
                        targetGroups = parseGroups(groupElement, entry.variables);
                    }

                    float px = 0, py = 0, pz = 0;
                    float rx = 0, ry = 0, rz = 0;
                    float sx = 1, sy = 1, sz = 1;
                    String deformGroup = currentDeformGroup;
                    String deformMaterialBase = currentDeformMaterialBase;
                    String deformMaterialDamaged = currentDeformMaterialDamaged;

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
                            deformGroup = getStringSafe(trans, "deformGroup", deformGroup);
                            deformMaterialBase = getStringSafe(
                                    trans, "deformMaterialBase", deformMaterialBase);
                            deformMaterialDamaged = getStringSafe(
                                    trans, "deformMaterialDamaged", deformMaterialDamaged);
                        }
                    }

                    vehicle.flexbodies.registerFlexbody(
                            meshName, rootPartName, targetGroups,
                            deformGroup, deformMaterialBase, deformMaterialDamaged,
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
