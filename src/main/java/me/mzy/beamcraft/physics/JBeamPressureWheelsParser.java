package me.mzy.beamcraft.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.HashMap;
import java.util.Map;

/**
 * 根据 BeamNG Pressure Wheels 文档 (v0.38.5+) 完整实现参数解析。
 * 所有压力轮参数均从 JSON 中读取并存储，供后续物理生成使用。
 */
public class JBeamPressureWheelsParser {

    private static final Map<String, String> activeConfig = new HashMap<>();
    public static final Map<String, Double> globalVariables = new HashMap<>();

    public static void parsePressureWheels(JsonArray pressureWheels, SoftBodyVehicle vehicle) {
        boolean isHeader = true;
        for (JsonElement element : pressureWheels) {
            // 1. 状态修饰符 {...} → 更新活跃配置黑板
            if (element.isJsonObject()) {
                JsonObject mod = element.getAsJsonObject();
                for (String key : mod.keySet()) {
                    JsonElement val = mod.get(key);
                    if (val.isJsonPrimitive()) {
                        activeConfig.put(key, val.getAsString());
                    }
                }
                continue;
            }

            // 2. 数据行 [...] → 触发生成指令
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() < 5) continue;

                String wheelName = row.get(0).getAsString();
                Integer n1 = vehicle.nodes.nameToIndex.get(row.get(3).getAsString());
                Integer n2 = vehicle.nodes.nameToIndex.get(row.get(4).getAsString());
                if (n1 == null || n2 == null) continue;

                Integer nodeS = null;
                if (row.size() > 5 && !row.get(5).isJsonNull() && !row.get(5).getAsString().equals("9999")) {
                    nodeS = vehicle.nodes.nameToIndex.get(row.get(5).getAsString());
                }
                Integer nodeArm = null;
                if (row.size() > 6 && !row.get(6).isJsonNull() && !row.get(6).getAsString().equals("9999")) {
                    nodeArm = vehicle.nodes.nameToIndex.get(row.get(6).getAsString());
                }
                int wheelDir = 1;
                if (row.size() > 7 && !row.get(7).isJsonNull() && !row.get(7).getAsString().equals("9999")) {
                    try { wheelDir = (int) Double.parseDouble(row.get(7).getAsString()); } catch (Exception ignored) {}
                }

                // ========== 以下读取文档中所有可选参数（即使暂不使用） ==========

                // ----- 基础开关与尺寸 -----
                boolean hasTire = getBool("hasTire", true);
                int numRays = (int) getVal("numRays", 12.0);
                double wheelOffset = getVal("wheelOffset", 0.0);
                double radius = getVal("radius", 0.35);
                double tireWidth = getVal("tireWidth", 0.2);
                String speedo = getStr("speedo", null);
                double propulsed = getVal("propulsed", 0.0);
                boolean selfCollision = getBool("selfCollision", false);
                boolean collision = getBool("collision", true);
                boolean disableMeshBreaking = getBool("disableMeshBreaking", false);
                boolean disableHubMeshBreaking = getBool("disableHubMeshBreaking", false);
                String axleBeams = getStr("axleBeams", null);      // 实际为数组，此处只读字符串形式

                // ----- 轮毂参数 -----
                double hubRadius = getVal("hubRadius", 0.2);
                double hubWidth = getVal("hubWidth", 0.2);
                double hubNodeWeight = getFirstVal("hubNodeWeight", "hubWeight", 0.5);
                double hubFrictionCoef = getFirstVal("hubFrictionCoef", "frictionCoef", 0.5);
                String hubNodeMaterial = getStr("hubNodeMaterial", "METAL");
                // hub 各种梁参数
                double hubBeamSpring = getVal("hubBeamSpring", 251000);
                double hubBeamDamp = getVal("hubBeamDamp", 5);
                double hubBeamDeform = getVal("hubBeamDeform", 40000);
                double hubBeamStrength = getVal("hubBeamStrength", 160000);
                double hubTreadBeamSpring = getVal("hubTreadBeamSpring", 901000);
                double hubTreadBeamDamp = getVal("hubTreadBeamDamp", 6);
                double hubPeripheryBeamSpring = getVal("hubPeripheryBeamSpring", 901000);
                double hubPeripheryBeamDamp = getVal("hubPeripheryBeamDamp", 6);
                double hubSideBeamSpring = getVal("hubSideBeamSpring", 1351000);
                double hubSideBeamDamp = getVal("hubSideBeamDamp", 6);
                double hubReinfBeamSpring = getVal("hubReinfBeamSpring", 0);  // 简化模型使用
                double hubReinfBeamDamp = getVal("hubReinfBeamDamp", 0);

                // ----- 轮胎参数 -----
                double tireNodeWeight = getFirstVal("nodeWeight", "tireWeight", 0.15);
                double tireFrictionCoef = getFirstVal("frictionCoeff", "frictionCoef", 1.0);
                double slidingFrictionCoef = getVal("slidingFrictionCoeff", 1.0);
                double stribeckVelMult = getVal("stribeckVelMult", 1.0);
                double stribeckExponent = getVal("stribeckExponent", 1.75);
                double treadCoef = getVal("treadCoeff", 0.7);
                double noLoadCoef = getVal("noLoadCoeff", 1.28);
                double loadSensitivitySlope = getVal("loadSensitivitySlope", 0.00019);
                double fullLoadCoef = getVal("fullLoadCoeff", 0.4);
                double softnessCoef = getVal("softnessCoeff", 0.6);
                String nodeMaterial = getStr("nodeMaterial", "RUBBER");
                double pressurePSI = getVal("pressurePSI", 30.0);
                double maxPressurePSI = getVal("maxPressurePSI", 60.0);
                double dragCoef = getVal("dragCoef", 5.0);
                double skinDragCoef = getVal("skinDragCoef", 0.0);
                boolean triangleCollision = getBool("triangleCollision", false);
                boolean treadTriangleCollision = getBool("treadTriangleCollision", false);
                boolean side1TriangleCollision = getBool("side1TriangleCollision", false);
                boolean side2TriangleCollision = getBool("side2TriangleCollision", false);
                boolean hubTriangleCollision = getBool("hubTriangleCollision", false);
                boolean hubSide1TriangleCollision = getBool("hubSide1TriangleCollision", false);
                boolean hubSide2TriangleCollision = getBool("hubSide2TriangleCollision", false);

                // 轮胎梁参数（各向异性）
                double wheelSideBeamSpring = getVal("wheelSideBeamSpring", 15000);
                double wheelSideBeamDamp = getVal("wheelSideBeamDamp", 30);
                double wheelSideBeamSpringExpansion = getVal("wheelSideBeamSpringExpansion", 281000);
                double wheelSideBeamDampExpansion = getVal("wheelSideBeamDampExpansion", 30);
                double wheelSideTransitionZone = getVal("wheelSideTransitionZone", 0);
                double wheelSideBeamDeform = getVal("wheelSideBeamDeform", 11000);
                double wheelSideBeamStrength = getVal("wheelSideBeamStrength", 15000);
                double wheelSideReinfBeamSpring = getVal("wheelSideReinfBeamSpring", 15000);
                double wheelSideReinfBeamDamp = getVal("wheelSideReinfBeamDamp", 30);
                double wheelSideReinfBeamSpringExpansion = getVal("wheelSideReinfBeamSpringExpansion", 281000);
                double wheelSideReinfBeamDampExpansion = getVal("wheelSideReinfBeamDampExpansion", 30);
                double wheelReinfBeamSpring = getFirstVal("wheelReinfBeamSpring", "wheelTreadReinfBeamSpring", 120000);
                double wheelReinfBeamDamp = getFirstVal("wheelReinfBeamDamp", "wheelTreadReinfBeamDamp", 40);
                double wheelReinfBeamDeform = getVal("wheelReinfBeamDeform", 220000);
                double wheelReinfBeamStrength = getVal("wheelReinfBeamStrength", PhysicsWorld.KINDA_BIG_NUMBER);
                double wheelTreadBeamSpring = getVal("wheelTreadBeamSpring", 50000);
                double wheelTreadBeamDamp = getVal("wheelTreadBeamDamp", 50);
                double wheelTreadBeamDeform = getVal("wheelTreadBeamDeform", 10000);
                double wheelTreadBeamStrength = getVal("wheelTreadBeamStrength", 13000);
                double wheelTreadReinfBeamSpring = getVal("wheelTreadReinfBeamSpring", 120000);
                double wheelTreadReinfBeamDamp = getVal("wheelTreadReinfBeamDamp", 40);
                double wheelPeripheryBeamSpring = getVal("wheelPeripheryBeamSpring", 35000);
                double wheelPeripheryBeamDamp = getVal("wheelPeripheryBeamDamp", 23);
                double wheelPeripheryBeamDeform = getVal("wheelPeripheryBeamDeform", 40000);
                double wheelPeripheryBeamStrength = getVal("wheelPeripheryBeamStrength", 40000);
                double wheelPeripheryReinfBeamSpring = getVal("wheelPeripheryReinfBeamSpring", 95000);
                double wheelPeripheryReinfBeamDamp = getVal("wheelPeripheryReinfBeamDamp", 23);
                boolean enableTireReinfBeams = getBool("enableTireReinfBeams", true);
                boolean enableTireLBeams = getBool("enableTireLBeams", true);
                boolean enableTireSideReinfBeams = getBool("enableTireSideReinfBeams", true);
                boolean enableTreadReinfBeams = getBool("enableTreadReinfBeams", true);
                boolean enableTirePeripheryReinfBeams = getBool("enableTirePeripheryReinfBeams", true);
                boolean enableTireSupportBeams = getBool("enableTireSupportBeams", false);
                double tireSupportBeamSpring = getVal("tireSupportBeamSpring", 0);
                double tireSupportBeamDamp = getVal("tireSupportBeamDamp", 0);

                // ----- 刹车参数 -----
                double brakeTorque = getVal("brakeTorque", 0);
                double parkingTorque = getVal("parkingTorque", 0);
                double brakeSpring = getVal("brakeSpring", 10);
                boolean enableBrakeThermals = getBool("enableBrakeThermals", false);
                double brakeDiameter = getVal("brakeDiameter", 0.35);
                double brakeMass = getVal("brakeMass", 10);
                String brakeType = getStr("brakeType", "vented-disc");
                String rotorMaterial = getStr("rotorMaterial", "steel");
                double brakeVentingCoef = getVal("brakeVentingCoeff", 1.0);
                String padMaterial = getStr("padMaterial", "basic");
                double brakeInputSplit = getVal("brakeInputSplit", 1.0);
                double brakeSplitCoef = getVal("brakeSplitCoef", 1.0);
                double squealCoefNatural = getVal("squealCoefNatural", 0);
                double squealCoefLowSpeed = getVal("squealCoefLowSpeed", 0);
                double squealCoefGlazing = getVal("squealCoefGlazing", 1);
                boolean enableABS = getBool("enableABS", false);
                double absSlipRatioTarget = getVal("absSlipRatioTarget", 0.18);
                double absHz = getVal("absHz", 100);
                double brakePressureInDelay = getVal("brakePressureInDelay", 0.05);
                double brakePressureOutDelay = getVal("brakePressureOutDelay", 0.1);

                // ----- 轮毂盖参数 -----
                boolean enableHubcaps = getBool("enableHubcaps", false);
                String hubcapBreakGroup = getStr("hubcapBreakGroup", null);
                String hubcapGroup = getStr("hubcapGroup", null);
                boolean hubcapCollision = getBool("hubcapCollision", false);
                boolean hubcapSelfCollision = getBool("hubcapSelfCollision", false);
                boolean enableExtraHubcapBeams = getBool("enableExtraHubcapBeams", false);
                double hubcapOffset = getVal("hubcapOffset", 0);
                double hubcapWidth = getVal("hubcapWidth", 0.06);
                double hubcapRadius = getVal("hubcapRadius", 0.11);
                double hubcapBeamSpring = getVal("hubcapBeamSpring", 121000);
                double hubcapBeamDamp = getVal("hubcapBeamDamp", 4);
                double hubcapBeamDeform = getVal("hubcapBeamDeform", 3500);
                double hubcapBeamStrength = getVal("hubcapBeamStrength", 15000);
                double hubcapAttachBeamSpring = getVal("hubcapAttachBeamSpring", 121000);
                double hubcapAttachBeamDamp = getVal("hubcapAttachBeamDamp", 8);
                double hubcapAttachBeamDeform = getVal("hubcapAttachBeamDeform", 1200);
                double hubcapAttachBeamStrength = getVal("hubcapAttachBeamStrength", 1800);
                double hubcapSupportBeamDeform = getVal("hubcapSupportBeamDeform", 2500);
                double hubcapSupportBeamStrength = getVal("hubcapSupportBeamStrength", 5000);
                double hubcapNodeWeight = getVal("hubcapNodeWeight", 0.06);
                double hubcapCenterNodeWeight = getVal("hubcapCenterNodeWeight", 0.06);
                String hubcapNodeMaterial = getStr("hubcapNodeMaterial", "METAL");
                double hubcapFrictionCoef = getVal("hubcapFrictionCoef", 0.7);

                // ----- 转向/传动高级节点 -----
                String steerAxisUp = getStr("steerAxisUp", null);
                String steerAxisDown = getStr("steerAxisDown", null);
                String torqueCoupling = getStr("torqueCoupling", null);
                String torqueArm = getStr("torqueArm", null);
                String torqueArm2 = getStr("torqueArm2", null);
                String torqueJointNode1 = getStr("torqueJointNode1", null);
                String torqueJointNode2 = getStr("torqueJointNode2", null);

                // 简化车辆专用
                double hubRadiusSimple = getVal("hubRadiusSimple", -1);

                vehicle.wheels.generateHub(
                        wheelName, n1, n2, nodeS, nodeArm, wheelDir, numRays,
                        hubRadius, hubWidth, wheelOffset,
                        hubNodeWeight, hubFrictionCoef,
                        hubBeamSpring, hubBeamDamp, hubBeamDeform, hubBeamStrength,
                        hubTreadBeamSpring, hubTreadBeamDamp,
                        hubPeripheryBeamSpring, hubPeripheryBeamDamp,
                        hubSideBeamSpring, hubSideBeamDamp,
                        hubReinfBeamSpring, hubReinfBeamDamp,
                        hubTriangleCollision, hubSide1TriangleCollision, hubSide2TriangleCollision,
                        hubNodeMaterial,
                        enableHubcaps, hubcapBreakGroup, hubcapGroup,
                        hubcapCollision, hubcapSelfCollision, enableExtraHubcapBeams,
                        hubcapOffset, hubcapWidth, hubcapRadius,
                        hubcapBeamSpring, hubcapBeamDamp, hubcapBeamDeform, hubcapBeamStrength,
                        hubcapAttachBeamSpring, hubcapAttachBeamDamp, hubcapAttachBeamDeform, hubcapAttachBeamStrength,
                        hubcapSupportBeamDeform, hubcapSupportBeamStrength,
                        hubcapNodeWeight, hubcapCenterNodeWeight, hubcapNodeMaterial, hubcapFrictionCoef,
                        hubRadiusSimple
                );

                if (hasTire) {
                    vehicle.wheels.generateTire(
                            wheelName, n1, n2, wheelDir, numRays,
                            radius, tireWidth, wheelOffset,
                            tireNodeWeight, tireFrictionCoef, pressurePSI,
                            slidingFrictionCoef, stribeckVelMult, stribeckExponent,
                            treadCoef, noLoadCoef, loadSensitivitySlope, fullLoadCoef,
                            softnessCoef, maxPressurePSI,
                            dragCoef, skinDragCoef,
                            wheelTreadBeamSpring, wheelTreadBeamDamp, wheelTreadBeamDeform, wheelTreadBeamStrength,
                            wheelPeripheryBeamSpring, wheelPeripheryBeamDamp, wheelPeripheryBeamDeform, wheelPeripheryBeamStrength,
                            wheelSideBeamSpring, wheelSideBeamDamp,
                            wheelSideBeamSpringExpansion, wheelSideBeamDampExpansion, wheelSideTransitionZone,
                            wheelSideBeamDeform, wheelSideBeamStrength,
                            wheelReinfBeamSpring, wheelReinfBeamDamp, wheelReinfBeamDeform, wheelReinfBeamStrength,
                            wheelTreadReinfBeamSpring, wheelTreadReinfBeamDamp,
                            wheelPeripheryReinfBeamSpring, wheelPeripheryReinfBeamDamp,
                            wheelSideReinfBeamSpring, wheelSideReinfBeamDamp, wheelSideReinfBeamSpringExpansion, wheelSideReinfBeamDampExpansion,
                            enableTireLBeams, enableTireReinfBeams, enableTireSideReinfBeams,
                            enableTreadReinfBeams, enableTirePeripheryReinfBeams, enableTireSupportBeams,
                            tireSupportBeamSpring, tireSupportBeamDamp,
                            triangleCollision, treadTriangleCollision, side1TriangleCollision, side2TriangleCollision,
                            nodeMaterial,
                            brakeTorque, parkingTorque, brakeSpring,
                            enableBrakeThermals, brakeDiameter, brakeMass,
                            brakeType, rotorMaterial, brakeVentingCoef, padMaterial,
                            brakeInputSplit, brakeSplitCoef,
                            squealCoefNatural, squealCoefLowSpeed, squealCoefGlazing,
                            enableABS, absSlipRatioTarget, absHz,
                            brakePressureInDelay, brakePressureOutDelay
                    );
                }
            }
        }
    }

    // ======================= 参数读取辅助函数 =======================

    private static double getVal(String key, double def) {
        if (!activeConfig.containsKey(key)) return def;
        String val = activeConfig.get(key);
        if (val.contains("$=")) {
            if (!globalVariables.containsKey("tirepressure_F")) globalVariables.put("tirepressure_F", 30.0);
            if (!globalVariables.containsKey("tirepressure_R")) globalVariables.put("tirepressure_R", 30.0);
            Double parsed = evaluateBeamNGExpression(val, globalVariables);
            return parsed != null ? parsed : def;
        }
        try { return Double.parseDouble(val); } catch (NumberFormatException e) { return def; }
    }

    private static String getStr(String key, String def) {
        if (!activeConfig.containsKey(key)) return def;
        return activeConfig.get(key);
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

    /**
     * 解析 BeamNG 风格的表达式，如 "$= $tirepressure_F * 550 + 10"
     * 支持 + - * / 和变量替换（变量以 $ 开头，未定义变量 → 0.0）
     */
    public static Double evaluateBeamNGExpression(String expr, Map<String, Double> variables) {
        expr = expr.trim();
        if (!expr.startsWith("$=")) {
            try { return Double.parseDouble(expr); } catch (Exception e) { return null; }
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
                    Double val = variables.getOrDefault(varName, 0.0);
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
                double left = Double.parseDouble(equation.substring(leftStart, opIdx));
                double right = Double.parseDouble(equation.substring(opIdx + 1, rightEnd));
                double res = (equation.charAt(opIdx) == '*') ? left * right : left / right;
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
                double left = Double.parseDouble(equation.substring(leftStart, opIdx));
                double right = Double.parseDouble(equation.substring(opIdx + 1, rightEnd));
                double res = (equation.charAt(opIdx) == '+') ? left + right : left - right;
                equation = equation.substring(0, leftStart) + res + equation.substring(rightEnd);
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