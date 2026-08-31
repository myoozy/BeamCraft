package me.mzy.beamcraft.client.physics.powertrain;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import me.mzy.beamcraft.client.physics.JBeamParser;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DevicePatchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DifferentialSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorsionReactorSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.UnsupportedConfig;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ValueModifier;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * 构建期 powertrain 解析层：把单个 part（已清洗为严格 JSON 的 JsonObject）里的
 * {@code powertrain} 表格解析成 {@link List}<{@link DeviceSpec}>。
 *
 * <p>遵循本仓库 JBeam 表格风格：表头 / 独立 {@code {...}} 配置修改器 / 行尾行内
 * {@code {...}} 对象 / {@code $=...} 表达式。与 BeamNG 官方加载器对齐的合并优先级
 * （高 → 低）：
 * <ol>
 *   <li>设备命名配置对象 {@code part["<name>"]}（BeamNG
 *       {@code tableMergeRecursive(row, part[name] or {})}，命名对象覆盖行内配置）；</li>
 *   <li>行尾行内对象（该行的 trailing {@code {...}}）；</li>
 *   <li>powertrain 数组中独立 {@code {...}} 修改器（黑板，作用于其后的所有行）；</li>
 *   <li>内置默认值。</li>
 * </ol>
 *
 * <p>每个配置键读取时支持 {@code $=...} / {@code $var} 表达式求值（委托给
 * {@link JBeamParser#getDoubleSafe} / {@link JBeamParser#evaluateBeamNGExpression}）。
 * 数组单元（如 torque 表、gearRatios）统一委托给 {@link JBeamParser#getDoubleCell} 求值。
 * {@code $*key} 等值修改器无法在单 part 内解析，原样记录到
 * {@link DeviceSpec#valueModifiers()}。
 *
 * <p>未知设备类型不会被静默丢弃，而是保留为 {@link UnsupportedConfig}。
 * part 没有 {@code powertrain} 段时返回空列表。
 */
public final class JBeamPowertrainParser {

    private JBeamPowertrainParser() {}

    /** 支持的变速箱类型（共享同一 gearRatios/friction/torqueLossCoef 结构）。 */
    private static final List<String> GEARBOX_TYPES =
            List.of("manualgearbox", "gearbox", "automaticgearbox", "sequentialgearbox", "dctgearbox");

    /**
     * 解析单个 part 的 powertrain。没有 powertrain 段时返回空列表。
     *
     * @param part      清洗过的 part JSON（含 {@code powertrain} 数组）
     * @param variables 该 part 上下文中的变量表（用于 {@code $=...} 表达式求值）
     */
    public static List<DeviceSpec> parsePart(JsonObject part, Map<String, Double> variables) {
        if (part == null) return List.of();
        JsonElement ptEl = part.get("powertrain");

        List<DeviceSpec> out = new ArrayList<>();
        if (ptEl == null || !ptEl.isJsonArray()) {
            collectNamedDevicePatches(part, variables, List.of(), out);
            return List.copyOf(out);
        }
        JsonObject blackboard = new JsonObject();
        boolean headerSeen = false;
        List<String> declaredNames = new ArrayList<>();

        for (JsonElement element : ptEl.getAsJsonArray()) {
            // 独立配置修改器 {...} → 更新黑板
            if (element.isJsonObject()) {
                JBeamParser.mergeJsonObjectsRecursive(blackboard, element.getAsJsonObject());
                continue;
            }
            if (!element.isJsonArray()) continue;

            JsonArray row = element.getAsJsonArray();
            if (!headerSeen) {
                headerSeen = true;
                // 首行是表头 ["type","name","inputName","inputIndex"] 时跳过；
                // 若没有表头则把首行当作真正的设备行处理。
                if (JBeamParser.isHeaderRow(row, "type")) continue;
            }

            DeviceSpec spec = parseRow(part, row, blackboard, variables);
            if (spec != null) {
                out.add(spec);
                declaredNames.add(spec.name());
            }
        }
        collectNamedDevicePatches(part, variables, declaredNames, out);
        return List.copyOf(out);
    }

    /** Captures cross-part named-device overrides such as a selectable final drive. */
    private static void collectNamedDevicePatches(JsonObject part, Map<String, Double> vars,
                                                  List<String> declaredNames, List<DeviceSpec> out) {
        for (Map.Entry<String, JsonElement> entry : part.entrySet()) {
            if (declaredNames.contains(entry.getKey()) || !entry.getValue().isJsonObject()) continue;
            List<ValueModifier> assignments = directAssignments(entry.getValue().getAsJsonObject(), vars);
            if (!assignments.isEmpty()) out.add(new DevicePatchSpec(entry.getKey(), assignments));
        }
    }

    private static List<ValueModifier> directAssignments(JsonObject cfg, Map<String, Double> vars) {
        List<ValueModifier> out = new ArrayList<>();
        for (String key : DIRECT_PATCH_FIELDS) {
            if (!cfg.has(key)) continue;
            double value = JBeamParser.getDoubleSafe(cfg, key, Double.NaN, vars);
            if (!Double.isNaN(value)) out.add(new ValueModifier(key, '=', value));
        }
        out.addAll(valueModifiers(cfg, vars));
        return out;
    }

    private static final List<String> DIRECT_PATCH_FIELDS = List.of(
            "gearRatio", "diffTorqueSplit", "friction", "dynamicFriction", "torqueLossCoef",
            "inertia", "idleRPM", "maxRPM", "engineBrakeTorque", "starterTorque",
            "starterMaxRPM", "starterRPM", "startRPM", "crankingRPM", "revLimiterRPM",
            "revLimiterCutTime", "revLimiterMaxRPMDrop", "revLimiterRPMChange",
            "lockTorque", "lockSpring", "lockSpringCoef", "lockDampRatio",
            "clutchFreePlay", "clutchStiffness", "gearChangeTime", "maxGearChangeTime",
            "dctClutchTime");

    // ---------------------------------------------------------------- row 解析

    private static DeviceSpec parseRow(JsonObject part, JsonArray row, JsonObject blackboard, Map<String, Double> vars) {
        if (row.size() < 2) return null; // 至少要有 type + name
        String type = JBeamParser.getStringCell(row.get(0));
        String name = JBeamParser.getStringCell(row.get(1));
        if (type == null || name == null) return null;

        String inputName = row.size() > 2 ? JBeamParser.getStringCell(row.get(2)) : null;
        int inputIndex = row.size() > 3 ? JBeamParser.getIntCell(row.get(3), 1) : 1;

        // 行尾行内对象（可能有多个，全部合并）
        JsonObject inline = new JsonObject();
        for (int i = 4; i < row.size(); i++) {
            if (row.get(i).isJsonObject()) JBeamParser.mergeJsonObjectsRecursive(inline, row.get(i).getAsJsonObject());
        }

        // 合并：黑板 → 行内 → 命名配置对象（命名对象优先级最高）
        JsonObject cfg = JBeamParser.copyJsonObject(blackboard);
        JBeamParser.mergeJsonObjectsRecursive(cfg, inline);
        JsonElement named = part.get(name);
        if (named instanceof JsonObject jo) JBeamParser.mergeJsonObjectsRecursive(cfg, jo);

        return buildDevice(type, name, inputName, inputIndex, cfg, vars);
    }

    private static DeviceSpec buildDevice(String type, String name, String inputName, int inputIndex,
                                          JsonObject cfg, Map<String, Double> vars) {
        String lower = type.toLowerCase();
        switch (lower) {
            case "combustionengine":
                return combustionEngine(type, name, inputName, inputIndex, cfg, vars);
            case "frictionclutch":
                return frictionClutch(type, name, inputName, inputIndex, cfg, vars);
            default:
                if (GEARBOX_TYPES.contains(lower)) {
                    return gearbox(type, name, inputName, inputIndex, cfg, vars);
                }
                switch (lower) {
                    case "shaft":
                        return shaft(type, name, inputName, inputIndex, cfg, vars);
                    case "torsionreactor":
                        return torsionReactor(type, name, inputName, inputIndex, cfg, vars);
                    case "differential":
                    case "opendifferential":
                        return differential(type, name, inputName, inputIndex, cfg, vars);
                    default:
                        return unsupported(type, name, inputName, inputIndex, cfg);
                }
        }
    }

    private static CombustionEngineSpec combustionEngine(String type, String name, String inputName, int inputIndex,
                                                         JsonObject cfg, Map<String, Double> vars) {
        double friction = d(cfg, "friction", 0.0, vars);
        // BeamNG 默认 engineBrakeTorque = friction * 2（未显式给出时）
        double engineBrakeTorque = cfg.has("engineBrakeTorque")
                ? d(cfg, "engineBrakeTorque", 0.0, vars)
                : friction * 2.0;
        double maxRPM = d(cfg, "maxRPM", 6000.0, vars);
        double revLimiterRPM = cfg.has("revLimiterRPM") ? d(cfg, "revLimiterRPM", 0.0, vars) : maxRPM;
        return new CombustionEngineSpec(
                type, name, inputName, inputIndex,
                d(cfg, "inertia", 0.1, vars),
                d(cfg, "idleRPM", 800.0, vars),
                maxRPM,
                friction,
                d(cfg, "dynamicFriction", 0.0, vars),
                engineBrakeTorque,
                torqueTable(cfg, "torque", vars),
                JBeamParser.getStringListSafe(cfg, vars, "torqueReactionNodes:", "torqueReactionNodes", "torqueReactionNodes_nodes"),
                valueModifiers(cfg, vars),
                d(cfg, "starterTorque", 0.0, vars),
                JBeamParser.getFirstDoubleSafe(cfg, 400.0, vars, "starterMaxRPM", "starterRPM", "startRPM"),
                d(cfg, "crankingRPM", 100.0, vars),
                revLimiterRPM,
                s(cfg, "revLimiterType", "time"),
                d(cfg, "revLimiterCutTime", 0.15, vars),
                cfg.has("revLimiterMaxRPMDrop") ? d(cfg, "revLimiterMaxRPMDrop", 0.0, vars)
                        : d(cfg, "revLimiterRPMChange", 300.0, vars)
        );
    }

    private static FrictionClutchSpec frictionClutch(String type, String name, String inputName, int inputIndex,
                                                     JsonObject cfg, Map<String, Double> vars) {
        return new FrictionClutchSpec(
                type, name, inputName, inputIndex,
                d(cfg, "lockTorque", 0.0, vars),
                d(cfg, "lockSpring", 0.0, vars),
                d(cfg, "lockSpringCoef", 1.0, vars),
                d(cfg, "lockDampRatio", 0.15, vars),
                d(cfg, "clutchFreePlay", 0.125, vars),
                d(cfg, "clutchStiffness", 1.0, vars),
                valueModifiers(cfg, vars)
        );
    }

    private static GearboxSpec gearbox(String type, String name, String inputName, int inputIndex,
                                       JsonObject cfg, Map<String, Double> vars) {
        double shiftTime = JBeamParser.getFirstDoubleSafe(
                cfg, -1.0, vars, "gearChangeTime", "maxGearChangeTime", "dctClutchTime");
        if (shiftTime <= 0.0) shiftTime = defaultShiftTime(type);
        return new GearboxSpec(
                type, name, inputName, inputIndex,
                JBeamParser.getDoubleListSafe(cfg, "gearRatios", vars),
                b(cfg, "fixedFirstGear", false),
                d(cfg, "friction", 0.0, vars),
                d(cfg, "dynamicFriction", 0.0, vars),
                d(cfg, "torqueLossCoef", 0.0, vars),
                valueModifiers(cfg, vars),
                shiftTime
        );
    }

    private static ShaftSpec shaft(String type, String name, String inputName, int inputIndex,
                                   JsonObject cfg, Map<String, Double> vars) {
        return new ShaftSpec(
                type, name, inputName, inputIndex,
                d(cfg, "gearRatio", 1.0, vars),
                s(cfg, "connectedWheel", null),
                d(cfg, "friction", 0.0, vars),
                d(cfg, "dynamicFriction", 0.0, vars),
                d(cfg, "torqueLossCoef", 0.0, vars),
                JBeamParser.getStringListSafe(cfg, vars, "torqueReactionNodes:", "torqueReactionNodes", "torqueReactionNodes_nodes"),
                JBeamParser.getIntListSafe(cfg, "outputPortOverride"),
                valueModifiers(cfg, vars)
        );
    }

    private static TorsionReactorSpec torsionReactor(String type, String name, String inputName, int inputIndex,
                                                     JsonObject cfg, Map<String, Double> vars) {
        return new TorsionReactorSpec(
                type, name, inputName, inputIndex,
                d(cfg, "gearRatio", 1.0, vars),
                s(cfg, "connectedWheel", null),
                d(cfg, "friction", 0.0, vars),
                d(cfg, "dynamicFriction", 0.0, vars),
                d(cfg, "torqueLossCoef", 0.0, vars),
                JBeamParser.getStringListSafe(cfg, vars, "torqueReactionNodes:", "torqueReactionNodes", "torqueReactionNodes_nodes"),
                JBeamParser.getIntListSafe(cfg, "outputPortOverride"),
                valueModifiers(cfg, vars)
        );
    }

    private static DifferentialSpec differential(String type, String name, String inputName, int inputIndex,
                                                JsonObject cfg, Map<String, Double> vars) {
        return new DifferentialSpec(
                type, name, inputName, inputIndex,
                d(cfg, "gearRatio", 1.0, vars),
                d(cfg, "diffTorqueSplit", 0.5, vars),
                d(cfg, "friction", 0.0, vars),
                d(cfg, "dynamicFriction", 0.0, vars),
                d(cfg, "torqueLossCoef", 0.0, vars),
                s(cfg, "diffType", "open"),
                valueModifiers(cfg, vars)
        );
    }

    private static UnsupportedConfig unsupported(String type, String name, String inputName, int inputIndex,
                                                 JsonObject cfg) {
        Map<String, String> snapshot = new LinkedHashMap<>();
        for (Map.Entry<String, JsonElement> e : cfg.entrySet()) {
            JsonElement v = e.getValue();
            if (v == null || v.isJsonNull()) continue;
            snapshot.put(e.getKey(), v.isJsonPrimitive() ? v.getAsString() : v.toString());
        }
        return new UnsupportedConfig(
                type, name, inputName, inputIndex,
                snapshot,
                "unsupported powertrain device type: " + type,
                valueModifiers(cfg, Map.of())
        );
    }

    // ---------------------------------------------------------------- 值提取 helper

    /** 表头单元格：第一格为 "type"（大小写不敏感）视为表头行。 */
    /** 原始字符串单元格（trim）；非 primitive 或空串 → null。 */
    /** 整型单元格；解析失败返回 def。 */
    /** 数值单元格：数字直接读；字符串支持 {@code $=...} / {@code $var} 表达式；失败返回 NaN。 */
    // ---------------------------------------------------------------- 配置键读取

    private static double d(JsonObject cfg, String key, double def, Map<String, Double> vars) {
        return JBeamParser.getDoubleSafe(cfg, key, def, vars);
    }

    /** 按变速箱类型给出合理的换挡时间默认值（秒）。 */
    private static double defaultShiftTime(String type) {
        String lower = type.toLowerCase();
        if (lower.contains("automatic") || lower.contains("dct")) return 0.4;
        return 0.25;
    }

    private static String s(JsonObject cfg, String key, String def) {
        return JBeamParser.getStringSafe(cfg, key, def);
    }

    private static boolean b(JsonObject cfg, String key, boolean def) {
        return JBeamParser.getBooleanSafe(cfg, key, def);
    }

    /** 字符串数组（node 列表），按给定 key 顺序取第一个存在者；条目 {@code $=} 表达式会求值。 */
    /** 数值数组（如 gearRatios），单个数值或数组皆可；NaN 单元格被跳过。 */
    /** 整型数组（如 outputPortOverride）。 */
    /**
     * 带表头的两列表格（如 {@code [["rpm","torque"],[0,0],...]}）→ {@link TorquePoint} 列表。
     * 首行为表头（第一格非数值）时跳过；每行取 col0=rpm、col1=torque。
     */
    private static List<TorquePoint> torqueTable(JsonObject cfg, String key, Map<String, Double> vars) {
        JsonElement el = cfg.get(key);
        if (el == null || !el.isJsonArray()) return new ArrayList<>();
        List<TorquePoint> out = new ArrayList<>();
        boolean first = true;
        for (JsonElement rowEl : el.getAsJsonArray()) {
            if (!rowEl.isJsonArray()) continue;
            JsonArray row = rowEl.getAsJsonArray();
            if (first) {
                first = false;
                if (row.size() >= 1 && row.get(0).isJsonPrimitive()
                        && !row.get(0).getAsJsonPrimitive().isNumber()) {
                    continue; // 表头 ["rpm","torque"]
                }
            }
            if (row.size() < 2) continue;
            double rpm = JBeamParser.getDoubleCell(row.get(0), Double.NaN, vars);
            double torque = JBeamParser.getDoubleCell(row.get(1), Double.NaN, vars);
            if (Double.isNaN(rpm) || Double.isNaN(torque)) continue;
            out.add(new TorquePoint(rpm, torque));
        }
        return out;
    }

    /** 收集 {@code $*} / {@code $+} / {@code $-} / {@code $/} / {@code $=} 值修改器。 */
    private static List<ValueModifier> valueModifiers(JsonObject cfg, Map<String, Double> vars) {
        List<ValueModifier> out = new ArrayList<>();
        for (String key : cfg.keySet()) {
            if (key.length() < 2 || key.charAt(0) != '$') continue;
            char op = key.charAt(1);
            if (op == '*' || op == '+' || op == '-' || op == '/' || op == '=') {
                out.add(new ValueModifier(key.substring(2), op, d(cfg, key, 0.0, vars)));
            }
        }
        return out;
    }

    // ---------------------------------------------------------------- JSON 合并

    /** 把 overlay 合并进 base：同名且都是对象时递归合并，否则 overlay 覆盖（BeamNG tableMergeRecursive 语义）。 */
}
