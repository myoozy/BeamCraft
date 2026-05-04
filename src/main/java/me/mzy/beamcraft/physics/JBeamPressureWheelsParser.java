package me.mzy.beamcraft.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.HashMap;
import java.util.Map;

public class JBeamPressureWheelsParser {

    private static final Map<String, String> activeConfig = new HashMap<>();
    public static final Map<String, Double> globalVariables = new HashMap<>();

    public static void parsePressureWheels(JsonArray pressureWheels, SoftBodyVehicle vehicle) {
        boolean isHeader = true;

        for (JsonElement element : pressureWheels) {

            // 1. 如果是状态修饰符 {...}，无脑更新黑板
            if (element.isJsonObject()) {
                JsonObject mod = element.getAsJsonObject();
                for (String key : mod.keySet()) {
                    JsonElement val = mod.get(key);
                    if (val.isJsonPrimitive()) {
                        // 统一存为 String，等到生成时再去解析 Double 或公式
                        activeConfig.put(key, val.getAsString());
                    }
                }
                continue;
            }

            // 2. 如果是数据行 [...]，触发生成指令！
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() < 5) continue;

                String wheelName = row.get(0).getAsString();

                // 查找 n1 和 n2 (这里如果找得到，说明之前的 nodeOffset 必须修复才能位置正确)
                Integer n1 = vehicle.nodes.nameToIndex.get(row.get(3).getAsString());
                Integer n2 = vehicle.nodes.nameToIndex.get(row.get(4).getAsString());
                if (n1 == null || n2 == null) continue;

                // 提取附加节点
                Integer nodeS = null;
                if (row.size() > 5 && !row.get(5).isJsonNull() && !row.get(5).getAsString().equals("9999")) {
                    nodeS = vehicle.nodes.nameToIndex.get(row.get(5).getAsString());
                }
                Integer nodeArm = null;
                if (row.size() > 6 && !row.get(6).isJsonNull() && !row.get(6).getAsString().equals("9999")) {
                    nodeArm = vehicle.nodes.nameToIndex.get(row.get(6).getAsString());
                }

                // 🚀 1. 提取基础设置
                boolean hasTire = getBool("hasTire", true);
                int rays = (int) getVal("numRays", 12.0);
                double offset = getVal("wheelOffset", 0.0);

                // 🚀 2. 提取 Hub (轮毂) 参数
                double hubR = getVal("hubRadius", 0.2);
                double hubW = getVal("hubWidth", 0.2);
                double hubMass = getFirstVal("hubNodeWeight", "nodeWeight", 0.5);
                double hubFric = getFirstVal("hubFrictionCoef", "frictionCoef", 0.5);

                // Hub 的三维刚度和阻尼 (Tread=周长, Periphery=横向交叉, Side=侧面辐条)
                double hubTreadS = getFirstVal("hubTreadBeamSpring", "wheelTreadBeamSpring", 1.5e6);
                double hubTreadD = getFirstVal("hubTreadBeamDamp", "wheelTreadBeamDamp", 15);
                double hubPeriS  = getFirstVal("hubPeripheryBeamSpring", "wheelPeripheryBeamSpring", 1.5e6);
                double hubPeriD  = getFirstVal("hubPeripheryBeamDamp", "wheelPeripheryBeamDamp", 15);
                double hubSideS  = getFirstVal("hubSideBeamSpring", "wheelSideBeamSpring", 1.5e6);
                double hubSideD  = getFirstVal("hubSideBeamDamp", "wheelSideBeamDamp", 15);

                // 提取 wheelDir
                int wheelDir = 1;
                if (row.size() > 7 && !row.get(7).isJsonNull() && !row.get(7).getAsString().equals("9999")) {
                    try { wheelDir = (int)Double.parseDouble(row.get(7).getAsString()); } catch (Exception e) {}
                }

                vehicle.wheels.generateHub(wheelName, n1, n2, nodeS, nodeArm, wheelDir, rays, hubR, hubW, offset, hubMass, hubFric, hubTreadS, hubTreadD, hubPeriS, hubPeriD, hubSideS, hubSideD);

                if (hasTire) {
                    double tireR = getVal("radius", 0.35);
                    double tireW = getVal("tireWidth", 0.2);
                    // 如果没单独指定轮胎质量/摩擦，退回使用 nodeWeight
                    double tireMass = getFirstVal("tireNodeWeight", "nodeWeight", 0.15);
                    double tireFric = getFirstVal("tireFrictionCoef", "frictionCoef", 1.0);
                    double press = getVal("pressurePSI", 30.0);

                    // 轮胎的三维刚度和阻尼
                    double tireTreadS = getVal("wheelTreadBeamSpring", 40000);
                    double tireTreadD = getVal("wheelTreadBeamDamp", 80);
                    double tirePeriS  = getVal("wheelPeripheryBeamSpring", 40000);
                    double tirePeriD  = getVal("wheelPeripheryBeamDamp", 40);
                    double tireSideS  = getVal("wheelSideBeamSpring", 15000);
                    double tireSideD  = getVal("wheelSideBeamDamp", 30);

                    // 轮胎加强筋 (Reinf)，优先读取通用 reinf，读不到则退回具体的 TreadReinf
                    double reinfS = getFirstVal("wheelReinfBeamSpring", "wheelTreadReinfBeamSpring", 20000);
                    double reinfD = getFirstVal("wheelReinfBeamDamp", "wheelTreadReinfBeamDamp", 180);

                    vehicle.wheels.generateTire(wheelName, n1, n2, wheelDir, rays, tireR, tireW, offset, tireMass, tireFric, press, tireTreadS, tireTreadD, tirePeriS, tirePeriD, tireSideS, tireSideD, reinfS, reinfD);
                }
            }
        }
    }

    // --- 极简的数据提取小助手 ---

    private static double getVal(String key, double def) {
        if (!activeConfig.containsKey(key)) return def;
        String val = activeConfig.get(key);
        // 如果包含公式符号 $=，交给你写的 evaluateBeamNGExpression 去处理
        if (val.contains("$=")) {
            // 如果还没有完善的变量注册表，至少要在这里兜底一个默认的 30.0 胎压
            if (!globalVariables.containsKey("tirepressure_F")) globalVariables.put("tirepressure_F", 30.0);
            if (!globalVariables.containsKey("tirepressure_R")) globalVariables.put("tirepressure_R", 30.0);
            Double parsed = evaluateBeamNGExpression(val, new HashMap<>());
            return parsed != null ? parsed : def;
        }
        try { return Double.parseDouble(val); } catch (Exception e) { return def; }
    }

    private static double getFirstVal(String key1, String key2, double def) {
        if (activeConfig.containsKey(key1)) return getVal(key1, def);
        if (activeConfig.containsKey(key2)) return getVal(key2, def);
        return def;
    }

    private static boolean getBool(String key, boolean def) {
        if (!activeConfig.containsKey(key)) return def;
        String val = activeConfig.get(key).toLowerCase();
        return val.equals("true") || val.equals("1");
    }

    public static Double evaluateBeamNGExpression(String expr, Map<String, Double> variables) {
        expr = expr.trim();
        if (!expr.startsWith("$=")) {
            try { return Double.parseDouble(expr); } catch (Exception e) { return null; }
        }
        String equation = expr.substring(2);

        // 替换已知变量
        for (Map.Entry<String, Double> entry : variables.entrySet()) {
            String varName = "$" + entry.getKey();
            if (equation.contains(varName)) {
                equation = equation.replace(varName, String.valueOf(entry.getValue()));
            }
        }
        // ⚠️ 关键修正：将残余的未定义变量直接替换为 0.0 (计算 offset 时极为重要)
        equation = equation.replaceAll("\\$[a-zA-Z0-9_]+", "0.0");

        try {
            equation = equation.replaceAll("\\s+", "");
            if (equation.contains("*")) {
                String[] parts = equation.split("\\*");
                return Double.parseDouble(parts[0]) * Double.parseDouble(parts[1]);
            }
            if (equation.contains("+")) {
                String[] parts = equation.split("\\+");
                return Double.parseDouble(parts[0]) + Double.parseDouble(parts[1]);
            }
            // 减法需要防负数干扰，简单处理
            if (equation.contains("-") && equation.lastIndexOf('-') > 0) {
                int idx = equation.lastIndexOf('-');
                return Double.parseDouble(equation.substring(0, idx)) - Double.parseDouble(equation.substring(idx + 1));
            }
            return Double.parseDouble(equation);
        } catch (Exception e) {
            return null;
        }
    }

    public static void resetBlackboard() {
        activeConfig.clear();
    }
}