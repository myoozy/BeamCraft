package me.mzy.beamcraft.physics;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.HashMap;
import java.util.Map;

public class PressureWheelsState {

    // 内部类：存储单个车轮组的属性
    public static class WheelState {
        public Boolean hasTire;
        public Double hubRadius, wheelOffset, hubWidth;
        public Double radius, tireWidth;
        public Integer numRays;
        public Double nodeWeight, frictionCoef;
        public Double pressurePSI;

        public Double treadS, treadD, periS, periD, sideS, sideD;
        public Double treadReinfS, treadReinfD, periReinfS, periReinfD, sideReinfS, sideReinfD;
        public Double reinfS, reinfD;

        public void updateFrom(JsonObject mod) {
            // 🚀 改为无条件覆盖，确保子零件（轮胎/轮辋）的属性最终生效
            Boolean ht = getBooleanOrNull(mod, "hasTire"); if (ht != null) hasTire = ht;
            Double r = getDoubleOrNull(mod, "radius"); if (r != null) radius = r;
            Double tw = getDoubleOrNull(mod, "tireWidth"); if (tw != null) tireWidth = tw;
            Double p = getDoubleOrNull(mod, "pressurePSI"); if (p != null) pressurePSI = p;

            Double hr = getDoubleOrNull(mod, "hubRadius"); if (hr != null) hubRadius = hr;
            Double hw = getDoubleOrNull(mod, "hubWidth"); if (hw != null) hubWidth = hw;
            Double wo = getDoubleOrNull(mod, "wheelOffset"); if (wo != null) wheelOffset = wo;

            Double rays = getDoubleOrNull(mod, "numRays"); if (rays != null) numRays = rays.intValue();

            Double nw = getFirstDouble(mod, "nodeWeight", "hubNodeWeight"); if (nw != null) nodeWeight = nw;
            Double fc = getFirstDouble(mod, "frictionCoef", "hubFrictionCoef"); if (fc != null) frictionCoef = fc;

            Double ts = getFirstDouble(mod, "wheelTreadBeamSpring", "hubTreadBeamSpring"); if (ts != null) treadS = ts;
            Double td = getFirstDouble(mod, "wheelTreadBeamDamp", "hubTreadBeamDamp"); if (td != null) treadD = td;
            Double trs = getDoubleOrNull(mod, "wheelTreadReinfBeamSpring"); if (trs != null) treadReinfS = trs;
            Double trd = getDoubleOrNull(mod, "wheelTreadReinfBeamDamp"); if (trd != null) treadReinfD = trd;

            Double ps = getFirstDouble(mod, "wheelPeripheryBeamSpring", "hubPeripheryBeamSpring"); if (ps != null) periS = ps;
            Double pd = getFirstDouble(mod, "wheelPeripheryBeamDamp", "hubPeripheryBeamDamp"); if (pd != null) periD = pd;
            Double prs = getDoubleOrNull(mod, "wheelPeripheryReinfBeamSpring"); if (prs != null) periReinfS = prs;
            Double prd = getDoubleOrNull(mod, "wheelPeripheryReinfBeamDamp"); if (prd != null) periReinfD = prd;

            Double ss = getFirstDouble(mod, "wheelSideBeamSpring", "hubSideBeamSpring"); if (ss != null) sideS = ss;
            Double sd = getFirstDouble(mod, "wheelSideBeamDamp", "hubSideBeamDamp"); if (sd != null) sideD = sd;
            Double srs = getDoubleOrNull(mod, "wheelSideReinfBeamSpring"); if (srs != null) sideReinfS = srs;
            Double srd = getDoubleOrNull(mod, "wheelSideReinfBeamDamp"); if (srd != null) sideReinfD = srd;

            Double rs = getDoubleOrNull(mod, "wheelReinfBeamSpring"); if (rs != null) reinfS = rs;
            Double rd = getDoubleOrNull(mod, "wheelReinfBeamDamp"); if (rd != null) reinfD = rd;
        }

        private Double getFirstDouble(JsonObject mod, String key1, String key2) {
            Double val = getDoubleOrNull(mod, key1);
            return val != null ? val : getDoubleOrNull(mod, key2);
        }
    }

    // 核心改造：使用 Map 存储不同车轮（或者叫 group）的状态
    // "default" 用于存储那些没有指明名字的全局兜底属性
    public Map<String, WheelState> states = new HashMap<>();
    public String currentGroupName = "default"; // 默认组

    public PressureWheelsState() {
        states.put("default", new WheelState());
    }

    // 更新当前组名
    public void setGroupName(String name) {
        if (name != null && !name.isEmpty() && !name.equals("null")) {
            this.currentGroupName = name;
            states.putIfAbsent(name, new WheelState());
        } else {
            this.currentGroupName = "default";
        }
    }

    // 将属性更新到当前激活的所有组
    public void updateCurrentState(JsonObject mod) {
        states.get(currentGroupName).updateFrom(mod);
    }

    // 获取某个组的合并状态 (优先取该组专属的，如果为 null 再去 default 里找)
    public WheelState getMergedState(String groupName) {
        WheelState gState = states.getOrDefault(groupName, new WheelState());
        WheelState dState = states.get("default");
        WheelState merged = new WheelState();

        merged.hasTire = gState.hasTire != null ? gState.hasTire : dState.hasTire;
        merged.radius = gState.radius != null ? gState.radius : dState.radius;
        merged.tireWidth = gState.tireWidth != null ? gState.tireWidth : dState.tireWidth;
        merged.pressurePSI = gState.pressurePSI != null ? gState.pressurePSI : dState.pressurePSI;
        merged.hubRadius = gState.hubRadius != null ? gState.hubRadius : dState.hubRadius;
        merged.hubWidth = gState.hubWidth != null ? gState.hubWidth : dState.hubWidth;
        merged.wheelOffset = gState.wheelOffset != null ? gState.wheelOffset : dState.wheelOffset;
        merged.numRays = gState.numRays != null ? gState.numRays : dState.numRays;
        merged.nodeWeight = gState.nodeWeight != null ? gState.nodeWeight : dState.nodeWeight;
        merged.frictionCoef = gState.frictionCoef != null ? gState.frictionCoef : dState.frictionCoef;

        merged.treadS = gState.treadS != null ? gState.treadS : dState.treadS;
        merged.treadD = gState.treadD != null ? gState.treadD : dState.treadD;
        merged.periS = gState.periS != null ? gState.periS : dState.periS;
        merged.periD = gState.periD != null ? gState.periD : dState.periD;
        merged.sideS = gState.sideS != null ? gState.sideS : dState.sideS;
        merged.sideD = gState.sideD != null ? gState.sideD : dState.sideD;

        return merged;
    }

    /**
     * 微型 BeamNG 公式解析器
     * 专门处理形如 "$=$tirepressure_F*350" 或 "$=$trackoffset_F+0.32" 的表达式
     */
    public static Double evaluateBeamNGExpression(String expr, Map<String, Double> variables) {
        expr = expr.trim();
        // 如果不是公式，直接尝试解析为数字
        if (!expr.startsWith("$=")) {
            try { return Double.parseDouble(expr); } catch (Exception e) { return null; }
        }

        // 剔除 "$=" 前缀
        String equation = expr.substring(2);

        // 1. 替换变量 (例如把 $tirepressure_F 替换为 30.0)
        // 使用贪婪匹配替换所有已知变量
        for (Map.Entry<String, Double> entry : variables.entrySet()) {
            String varName = "$" + entry.getKey();
            if (equation.contains(varName)) {
                equation = equation.replace(varName, String.valueOf(entry.getValue()));
            }
        }

        // 2. 清理残余的未定义变量 (直接当做 0 处理，或者给默认值)
        // 比如如果没定义 tirepressure_F，我们强制塞一个 30.0
        equation = equation.replaceAll("\\$[a-zA-Z0-9_]+", "30.0");

        // 3. 极简的四则运算解析 (仅支持一层乘除加减)
        try {
            return evaluateSimpleMath(equation);
        } catch (Exception e) {
            System.err.println("⚠️ 无法解析 JBeam 公式: " + expr);
            return null;
        }
    }

    // 极其暴力的字符串四则运算求值 (仅作基础实现，可替换为更高阶的库)
    private static double evaluateSimpleMath(String math) {
        math = math.replaceAll("\\s+", ""); // 去空格

        // 优先处理乘法
        if (math.contains("*")) {
            String[] parts = math.split("\\*");
            return Double.parseDouble(parts[0]) * Double.parseDouble(parts[1]);
        }
        // 处理加法
        if (math.contains("+")) {
            String[] parts = math.split("\\+");
            return Double.parseDouble(parts[0]) + Double.parseDouble(parts[1]);
        }
        // 减法和除法同理... (对于引擎开发，建议后续引入 exp4j 这种轻量级数学解析库)

        return Double.parseDouble(math);
    }

    // 🚀 微型公式解析器
    public static Double getDoubleOrNull(JsonObject obj, String key) {
        if (!obj.has(key)) return null;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return null;

        if (el.isJsonPrimitive()) {
            if (el.getAsJsonPrimitive().isNumber()) return el.getAsDouble();
            if (el.getAsJsonPrimitive().isString()) {
                String str = el.getAsString().trim();
                if (str.isEmpty()) return null;

                try {
                    return Double.parseDouble(str);
                } catch (NumberFormatException e) {
                    // 拦截 "$=$tirepressure_F*350" 这种 JBeam 专有公式
                    if (str.contains("*")) {
                        String[] parts = str.split("\\*");
                        if (parts.length == 2) {
                            try {
                                double mult = Double.parseDouble(parts[1].trim());
                                // 引擎尚未实现变量注册表，此处暂时注入 30 PSI 的基准倍率
                                return 30.0 * mult;
                            } catch (Exception ignored) {}
                        }
                    }
                    return null;
                }
            }
        }
        return null;
    }

    public static Boolean getBooleanOrNull(JsonObject obj, String key) {
        if (!obj.has(key)) return null;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return null;
        if (el.isJsonPrimitive()) {
            if (el.getAsJsonPrimitive().isBoolean()) return el.getAsBoolean();
            if (el.getAsJsonPrimitive().isString()) {
                String str = el.getAsString().trim();
                if (str.isEmpty()) return null;
                if (str.equalsIgnoreCase("true")) return true;
                if (str.equalsIgnoreCase("false")) return false;
            }
        }
        return null;
    }
}