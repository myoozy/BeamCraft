package me.mzy.beamcraft.client.physics;

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

    public static void parsePressureWheels(JsonArray pressureWheels, SoftBodyVehicle vehicle, JBeamAssembler.PartEntry entry, JsonObject activeConfig) {
        boolean isHeader = true;
        for (JsonElement element : pressureWheels) {
            // 1. 状态修饰符 {...} → 更新活跃配置黑板
            if (element.isJsonObject()) {
                JsonObject mod = element.getAsJsonObject();
                for (String key : mod.keySet()) {
                    activeConfig.add(key, mod.get(key));
                }
                continue;
            }

            // 2. 数据行 [...] → 触发生成指令
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() < 5) continue;

                String wheelName = row.get(0).getAsString();
                String hubGroup = row.get(1).getAsString();
                String group = row.get(2).getAsString();
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
                boolean hasTire = getBool(activeConfig, "hasTire", true);
                int numRays = (int) getVal(activeConfig, "numRays", 12.0, entry.variables);
                double wheelOffset = getVal(activeConfig, "wheelOffset", 0.0, entry.variables);
                double radius = getVal(activeConfig, "radius", 0.35, entry.variables);
                double tireWidth = getVal(activeConfig, "tireWidth", 0.2, entry.variables);
                String speedo = getStr(activeConfig, "speedo", null);
                double propulsed = getVal(activeConfig, "propulsed", 0.0, entry.variables);
                boolean selfCollision = getBool(activeConfig, "selfCollision", false);
                boolean collision = getBool(activeConfig, "collision", true);
                boolean disableMeshBreaking = getBool(activeConfig, "disableMeshBreaking", false);
                boolean disableHubMeshBreaking = getBool(activeConfig, "disableHubMeshBreaking", false);
                String axleBeams = getStr(activeConfig, "axleBeams", null);      // 实际为数组，此处只读字符串形式

                // ----- 轮毂参数 -----
                double hubRadius = getVal(activeConfig, "hubRadius", 0.2, entry.variables);
                double hubWidth = getVal(activeConfig, "hubWidth", 0.2, entry.variables);
                double hubNodeWeight = getFirstVal(activeConfig, "hubNodeWeight", "hubWeight", 0.5, entry.variables);
                double hubFrictionCoef = getFirstVal(activeConfig, "hubFrictionCoef", "frictionCoef", 0.5, entry.variables);
                String hubNodeMaterial = getStr(activeConfig, "hubNodeMaterial", "METAL");
                // hub 各种梁参数
                double hubBeamSpring = getVal(activeConfig, "hubBeamSpring", 251000, entry.variables);
                double hubBeamDamp = getVal(activeConfig, "hubBeamDamp", 5, entry.variables);
                double hubBeamDeform = getVal(activeConfig, "hubBeamDeform", 40000, entry.variables);
                double hubBeamStrength = getVal(activeConfig, "hubBeamStrength", 160000, entry.variables);
                double hubTreadBeamSpring = getVal(activeConfig, "hubTreadBeamSpring", 901000, entry.variables);
                double hubTreadBeamDamp = getVal(activeConfig, "hubTreadBeamDamp", 6, entry.variables);
                double hubPeripheryBeamSpring = getVal(activeConfig, "hubPeripheryBeamSpring", 901000, entry.variables);
                double hubPeripheryBeamDamp = getVal(activeConfig, "hubPeripheryBeamDamp", 6, entry.variables);
                double hubSideBeamSpring = getVal(activeConfig, "hubSideBeamSpring", 1351000, entry.variables);
                double hubSideBeamDamp = getVal(activeConfig, "hubSideBeamDamp", 6, entry.variables);
                double hubReinfBeamSpring = getVal(activeConfig, "hubReinfBeamSpring", 0, entry.variables);
                double hubReinfBeamDamp = getVal(activeConfig, "hubReinfBeamDamp", 0, entry.variables);

                // ----- 轮胎参数 -----
                double tireNodeWeight = getFirstVal(activeConfig, "nodeWeight", "tireWeight", 0.15, entry.variables);
                double tireFrictionCoef = getFirstVal(activeConfig, "frictionCoeff", "frictionCoef", 1.0, entry.variables);
                double slidingFrictionCoef = getVal(activeConfig, "slidingFrictionCoeff", 1.0, entry.variables);
                double stribeckVelMult = getVal(activeConfig, "stribeckVelMult", 1.0, entry.variables);
                double stribeckExponent = getVal(activeConfig, "stribeckExponent", 1.75, entry.variables);
                double treadCoef = getVal(activeConfig, "treadCoeff", 1.0, entry.variables);
                double noLoadCoef = getVal(activeConfig, "noLoadCoeff", 1.28, entry.variables);
                double loadSensitivitySlope = getVal(activeConfig, "loadSensitivitySlope", 0.00019, entry.variables);
                double fullLoadCoef = getVal(activeConfig, "fullLoadCoeff", 0.4, entry.variables);
                double softnessCoef = getVal(activeConfig, "softnessCoeff", 0.6, entry.variables);
                String nodeMaterial = getStr(activeConfig, "nodeMaterial", "RUBBER");
                double pressurePSI = getVal(activeConfig, "pressurePSI", 30.0, entry.variables);
                double maxPressurePSI = getVal(activeConfig, "maxPressurePSI", 60.0, entry.variables);
                double dragCoef = getVal(activeConfig, "dragCoef", 5.0, entry.variables);
                double skinDragCoef = getVal(activeConfig, "skinDragCoef", 0.0, entry.variables);
                boolean triangleCollision = getBool(activeConfig, "triangleCollision", false);
                boolean treadTriangleCollision = getBool(activeConfig, "treadTriangleCollision", false);
                boolean side1TriangleCollision = getBool(activeConfig, "side1TriangleCollision", false);
                boolean side2TriangleCollision = getBool(activeConfig, "side2TriangleCollision", false);
                boolean hubTriangleCollision = getBool(activeConfig, "hubTriangleCollision", false);
                boolean hubSide1TriangleCollision = getBool(activeConfig, "hubSide1TriangleCollision", false);
                boolean hubSide2TriangleCollision = getBool(activeConfig, "hubSide2TriangleCollision", false);

                // 轮胎梁参数（各向异性）
                double wheelSideBeamSpring = getVal(activeConfig, "wheelSideBeamSpring", 15000, entry.variables);
                double wheelSideBeamDamp = getVal(activeConfig, "wheelSideBeamDamp", 30, entry.variables);
                double wheelSideBeamSpringExpansion = getVal(activeConfig, "wheelSideBeamSpringExpansion", 281000, entry.variables);
                double wheelSideBeamDampExpansion = getVal(activeConfig, "wheelSideBeamDampExpansion", 30, entry.variables);
                double wheelSideTransitionZone = getVal(activeConfig, "wheelSideTransitionZone", 0, entry.variables);
                double wheelSideBeamDeform = getVal(activeConfig, "wheelSideBeamDeform", 11000, entry.variables);
                double wheelSideBeamStrength = getVal(activeConfig, "wheelSideBeamStrength", 15000, entry.variables);
                double wheelSideReinfBeamSpring = getVal(activeConfig, "wheelSideReinfBeamSpring", 15000, entry.variables);
                double wheelSideReinfBeamDamp = getVal(activeConfig, "wheelSideReinfBeamDamp", 30, entry.variables);
                double wheelSideReinfBeamSpringExpansion = getVal(activeConfig, "wheelSideReinfBeamSpringExpansion", 281000, entry.variables);
                double wheelSideReinfBeamDampExpansion = getVal(activeConfig, "wheelSideReinfBeamDampExpansion", 30, entry.variables);
                double wheelReinfBeamSpring = getFirstVal(activeConfig, "wheelReinfBeamSpring", "wheelTreadReinfBeamSpring", 120000, entry.variables);
                double wheelReinfBeamDamp = getFirstVal(activeConfig, "wheelReinfBeamDamp", "wheelTreadReinfBeamDamp", 40, entry.variables);
                double wheelReinfBeamDeform = getVal(activeConfig, "wheelReinfBeamDeform", 220000, entry.variables);
                double wheelReinfBeamStrength = getVal(activeConfig, "wheelReinfBeamStrength", PhysicsWorld.KINDA_BIG_NUMBER, entry.variables);
                double wheelTreadBeamSpring = getVal(activeConfig, "wheelTreadBeamSpring", 50000, entry.variables);
                double wheelTreadBeamDamp = getVal(activeConfig, "wheelTreadBeamDamp", 50, entry.variables);
                double wheelTreadBeamDeform = getVal(activeConfig, "wheelTreadBeamDeform", 10000, entry.variables);
                double wheelTreadBeamStrength = getVal(activeConfig, "wheelTreadBeamStrength", 13000, entry.variables);
                double wheelTreadReinfBeamSpring = getVal(activeConfig, "wheelTreadReinfBeamSpring", 120000, entry.variables);
                double wheelTreadReinfBeamDamp = getVal(activeConfig, "wheelTreadReinfBeamDamp", 40, entry.variables);
                double wheelPeripheryBeamSpring = getVal(activeConfig, "wheelPeripheryBeamSpring", 35000, entry.variables);
                double wheelPeripheryBeamDamp = getVal(activeConfig, "wheelPeripheryBeamDamp", 23, entry.variables);
                double wheelPeripheryBeamDeform = getVal(activeConfig, "wheelPeripheryBeamDeform", 40000, entry.variables);
                double wheelPeripheryBeamStrength = getVal(activeConfig, "wheelPeripheryBeamStrength", 40000, entry.variables);
                double wheelPeripheryReinfBeamSpring = getVal(activeConfig, "wheelPeripheryReinfBeamSpring", 95000, entry.variables);
                double wheelPeripheryReinfBeamDamp = getVal(activeConfig, "wheelPeripheryReinfBeamDamp", 23, entry.variables);
                boolean enableTireReinfBeams = getBool(activeConfig, "enableTireReinfBeams", false);
                boolean enableTireLBeams = getBool(activeConfig, "enableTireLBeams", true);
                boolean enableTireSideReinfBeams = getBool(activeConfig, "enableTireSideReinfBeams", true);
                boolean enableTreadReinfBeams = getBool(activeConfig, "enableTreadReinfBeams", true);
                boolean enableTirePeripheryReinfBeams = getBool(activeConfig, "enableTirePeripheryReinfBeams", true);
                boolean enableTireSupportBeams = getBool(activeConfig, "enableTireSupportBeams", false);
                double tireSupportBeamSpring = getVal(activeConfig, "tireSupportBeamSpring", 0, entry.variables);
                double tireSupportBeamDamp = getVal(activeConfig, "tireSupportBeamDamp", 0, entry.variables);

                // ----- 刹车参数 -----
                double brakeTorque = getVal(activeConfig, "brakeTorque", 0, entry.variables);
                double parkingTorque = getVal(activeConfig, "parkingTorque", 0, entry.variables);
                double brakeSpring = getVal(activeConfig, "brakeSpring", 10, entry.variables);
                boolean enableBrakeThermals = getBool(activeConfig, "enableBrakeThermals", false);
                double brakeDiameter = getVal(activeConfig, "brakeDiameter", 0.35, entry.variables);
                double brakeMass = getVal(activeConfig, "brakeMass", 10, entry.variables);
                String brakeType = getStr(activeConfig, "brakeType", "vented-disc");
                String rotorMaterial = getStr(activeConfig, "rotorMaterial", "steel");
                double brakeVentingCoef = getVal(activeConfig, "brakeVentingCoeff", 1.0, entry.variables);
                String padMaterial = getStr(activeConfig, "padMaterial", "basic");
                double brakeInputSplit = getVal(activeConfig, "brakeInputSplit", 1.0, entry.variables);
                double brakeSplitCoef = getVal(activeConfig, "brakeSplitCoef", 1.0, entry.variables);
                double squealCoefNatural = getVal(activeConfig, "squealCoefNatural", 0, entry.variables);
                double squealCoefLowSpeed = getVal(activeConfig, "squealCoefLowSpeed", 0, entry.variables);
                double squealCoefGlazing = getVal(activeConfig, "squealCoefGlazing", 1, entry.variables);
                boolean enableABS = getBool(activeConfig, "enableABS", false);
                double absSlipRatioTarget = getVal(activeConfig, "absSlipRatioTarget", 0.18, entry.variables);
                double absHz = getVal(activeConfig, "absHz", 100, entry.variables);
                double brakePressureInDelay = getVal(activeConfig, "brakePressureInDelay", 0.04, entry.variables);
                double brakePressureOutDelay = getVal(activeConfig, "brakePressureOutDelay", 0.04, entry.variables);

                // ----- 轮毂盖参数 -----
                boolean enableHubcaps = getBool(activeConfig, "enableHubcaps", false);
                String hubcapBreakGroup = getStr(activeConfig, "hubcapBreakGroup", null);
                String hubcapGroup = getStr(activeConfig, "hubcapGroup", null);
                boolean hubcapCollision = getBool(activeConfig, "hubcapCollision", false);
                boolean hubcapSelfCollision = getBool(activeConfig, "hubcapSelfCollision", false);
                boolean enableExtraHubcapBeams = getBool(activeConfig, "enableExtraHubcapBeams", false);
                double hubcapOffset = getVal(activeConfig, "hubcapOffset", 0, entry.variables);
                double hubcapWidth = getVal(activeConfig, "hubcapWidth", 0.06, entry.variables);
                double hubcapRadius = getVal(activeConfig, "hubcapRadius", 0.11, entry.variables);
                double hubcapBeamSpring = getVal(activeConfig, "hubcapBeamSpring", 121000, entry.variables);
                double hubcapBeamDamp = getVal(activeConfig, "hubcapBeamDamp", 4, entry.variables);
                double hubcapBeamDeform = getVal(activeConfig, "hubcapBeamDeform", 3500, entry.variables);
                double hubcapBeamStrength = getVal(activeConfig, "hubcapBeamStrength", 15000, entry.variables);
                double hubcapAttachBeamSpring = getVal(activeConfig, "hubcapAttachBeamSpring", 121000, entry.variables);
                double hubcapAttachBeamDamp = getVal(activeConfig, "hubcapAttachBeamDamp", 8, entry.variables);
                double hubcapAttachBeamDeform = getVal(activeConfig, "hubcapAttachBeamDeform", 1200, entry.variables);
                double hubcapAttachBeamStrength = getVal(activeConfig, "hubcapAttachBeamStrength", 1800, entry.variables);
                double hubcapSupportBeamDeform = getVal(activeConfig, "hubcapSupportBeamDeform", 2500, entry.variables);
                double hubcapSupportBeamStrength = getVal(activeConfig, "hubcapSupportBeamStrength", 5000, entry.variables);
                double hubcapNodeWeight = getVal(activeConfig, "hubcapNodeWeight", 0.06, entry.variables);
                double hubcapCenterNodeWeight = getVal(activeConfig, "hubcapCenterNodeWeight", 0.06, entry.variables);
                String hubcapNodeMaterial = getStr(activeConfig, "hubcapNodeMaterial", "METAL");
                double hubcapFrictionCoef = getVal(activeConfig, "hubcapFrictionCoef", 0.7, entry.variables);

                // ----- 转向/传动高级节点 -----
                String steerAxisUp = getStr(activeConfig, "steerAxisUp", null);
                String steerAxisDown = getStr(activeConfig, "steerAxisDown", null);
                // Drivetrain counter-torque is scoped per wheel row (usually inline in BeamNG
                // data), so reaction fields are read from the row's own object literals first,
                // falling back to the accumulated wheel state. Reading supports BeamNG's
                // trailing-colon key spelling ("torqueCoupling:").
                String torqueCoupling = getRowReactionName(activeConfig, row, "torqueCoupling", null);
                String torqueArm = getRowReactionName(activeConfig, row, "torqueArm", null);
                String torqueArm2 = getRowReactionName(activeConfig, row, "torqueArm2", null);
                String nodeCoupling = getRowReactionName(activeConfig, row, "nodeCoupling", null);
                String torqueJointNode1 = getStr(activeConfig, "torqueJointNode1", null);
                String torqueJointNode2 = getStr(activeConfig, "torqueJointNode2", null);

                // 简化车辆专用
                double hubRadiusSimple = getVal(activeConfig, "hubRadiusSimple", -1, entry.variables);

                vehicle.wheels.generateHub(new PhysicsSpecs.WheelHubSpec(
                        wheelName, n1, n2, nodeS, nodeArm, wheelDir, numRays,
                        hubRadius, hubWidth, wheelOffset,
                        hubNodeWeight, hubFrictionCoef,
                        hubBeamSpring, hubBeamDamp, hubBeamDeform, hubBeamStrength,
                        hubTreadBeamSpring, hubTreadBeamDamp,
                        hubPeripheryBeamSpring, hubPeripheryBeamDamp,
                        hubSideBeamSpring, hubSideBeamDamp,
                        hubReinfBeamSpring, hubReinfBeamDamp,
                        hubTriangleCollision, hubSide1TriangleCollision, hubSide2TriangleCollision,
                        hubNodeMaterial, hubGroup,
                        enableHubcaps, hubcapBreakGroup, hubcapGroup,
                        hubcapCollision, hubcapSelfCollision, enableExtraHubcapBeams,
                        hubcapOffset, hubcapWidth, hubcapRadius,
                        hubcapBeamSpring, hubcapBeamDamp, hubcapBeamDeform, hubcapBeamStrength,
                        hubcapAttachBeamSpring, hubcapAttachBeamDamp, hubcapAttachBeamDeform, hubcapAttachBeamStrength,
                        hubcapSupportBeamDeform, hubcapSupportBeamStrength,
                        hubcapNodeWeight, hubcapCenterNodeWeight, hubcapNodeMaterial, hubcapFrictionCoef,
                        hubRadiusSimple
                ));

                // Store the BeamNG pressure-wheel counter-torque nodes on the slot this hub
                // just occupied. The reaction fires only at apply time and only when both
                // torqueCoupling and torqueArm are present (per the BeamNG documentation);
                // torqueArm2 falls back to the inner axle node, nodeCoupling to the inner
                // axle node too. Unresolved / absent names resolve to -1 and stay inert.
                Integer wheelIndex = vehicle.wheels.nameToIndex.get(wheelName);
                if (wheelIndex != null) {
                    vehicle.wheels.setReactionNodes(
                            wheelIndex,
                            resolveNodeIndex(vehicle, torqueCoupling),
                            resolveNodeIndex(vehicle, torqueArm),
                            resolveNodeIndex(vehicle, torqueArm2));
                    vehicle.wheels.setBrakeCouplingNode(
                            wheelIndex,
                            resolveNodeIndex(vehicle, nodeCoupling));
                }

                if (hasTire) {
                    vehicle.wheels.generateTire(new PhysicsSpecs.WheelTireSpec(
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
                            nodeMaterial, group,
                            brakeTorque, parkingTorque, brakeSpring,
                            enableBrakeThermals, brakeDiameter, brakeMass,
                            brakeType, rotorMaterial, brakeVentingCoef, padMaterial,
                            brakeInputSplit, brakeSplitCoef,
                            squealCoefNatural, squealCoefLowSpeed, squealCoefGlazing,
                            enableABS, absSlipRatioTarget, absHz,
                            brakePressureInDelay, brakePressureOutDelay
                    ));
                }
            }
        }
    }

    // ======================= 参数读取辅助函数 =======================

    private static double getVal(JsonObject config, String key, double def, Map<String, Double> vars) {
        return JBeamParser.getDoubleSafe(config, key, def, vars);
    }

    private static String getStr(JsonObject config, String key, String def) {
        return JBeamParser.getStringSafe(config, key, def);
    }

    private static double getFirstVal(JsonObject config, String key1, String key2, double def, Map<String, Double> vars) {
        return JBeamParser.getFirstDoubleSafe(config, key1, key2, def, vars);
    }

    private static boolean getBool(JsonObject config, String key, boolean def) {
        return JBeamParser.getBooleanSafe(config, key, def);
    }

    /**
     * Reads a per-wheel node-name reaction field (torqueCoupling/torqueArm/torqueArm2/
     * nodeCoupling). BeamNG scopes these to the wheel row, typically as an object literal
     * appended to that row, so row-scoped values take precedence over the accumulated wheel
     * state. Both plain keys and BeamNG's trailing-colon spellings ("torqueCoupling:") are
     * honoured.
     */
    private static String getRowReactionName(JsonObject activeConfig, JsonArray row, String key, String def) {
        for (int i = 8; i < row.size(); i++) {
            if (!row.get(i).isJsonObject()) continue;
            JsonObject rowModifier = row.get(i).getAsJsonObject();
            JsonElement value = rowModifier.get(key);
            if (value == null) value = rowModifier.get(key + ":");
            if (value != null && !value.isJsonNull()) {
                try { return value.getAsString(); } catch (Exception ignored) { return def; }
            }
        }
        JsonElement value = activeConfig.get(key);
        if (value == null) value = activeConfig.get(key + ":");
        if (value == null || value.isJsonNull()) return def;
        try { return value.getAsString(); } catch (Exception ignored) { return def; }
    }

    /** Resolves a node name to its NodeContainer index, or -1 when absent/invalid. */
    private static int resolveNodeIndex(SoftBodyVehicle vehicle, String nodeName) {
        if (nodeName == null || nodeName.isEmpty()) return -1;
        Integer index = vehicle.nodes.nameToIndex.get(nodeName);
        return index != null ? index : -1;
    }
}
