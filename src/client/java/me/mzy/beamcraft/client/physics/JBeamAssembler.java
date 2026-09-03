package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import me.mzy.beamcraft.client.physics.powertrain.JBeamPowertrainParser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Assembles a complete physical vehicle from JBeam part definitions
 * Uses a two-pass assembly process to ensure valid node connections before structural elements
 */
public class JBeamAssembler {
    private int currentPartId = 0;

    /**
     * 封装级联空间变换的上下文环境
     * 记录累积的平移与旋转变换，支持 nodeRotate -> nodeOffset -> nodeMove 规范
     */
    public static class TransformContext {
        // 累积的总绝对平移矩阵/向量 (处理 nodeMove 和旋转后的累积位置)
        public double posX = 0.0, posY = 0.0, posZ = 0.0;
        // 累积的镜像平移量 (专用于处理 nodeOffset 的 X 轴对称特性)
        public double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
        // 累积欧拉角 (度数)
        public double rotX = 0.0, rotY = 0.0, rotZ = 0.0;

        public TransformContext() {}

        public TransformContext(TransformContext parent) {
            this.posX = parent.posX;
            this.posY = parent.posY;
            this.posZ = parent.posZ;
            this.offsetX = parent.offsetX;
            this.offsetY = parent.offsetY;
            this.offsetZ = parent.offsetZ;
            this.rotX = parent.rotX;
            this.rotY = parent.rotY;
            this.rotZ = parent.rotZ;
        }

        /**
         * 对输入坐标执行完整的局部变换计算
         */
        public double[] transformNode(double x, double y, double z) {
            // 1. 应用镜像 nodeOffset (若原始 X 在右侧即负值，则 X 轴 offset 取反)
            double appliedOffX = (x < 0.0) ? -offsetX : offsetX;
            double lx = x + appliedOffX;
            double ly = y + offsetY;
            double lz = z + offsetZ;

            // 2. 应用累积旋转 (绕原点或指定轴系旋转)
            double radX = Math.toRadians(rotX);
            double radY = Math.toRadians(rotY);
            double radZ = Math.toRadians(rotZ);

            // 按照 Z -> Y -> X 的内禀顺序旋转 (BeamNG 默认标准)
            // 绕 Z 轴
            double x1 = lx * Math.cos(radZ) - ly * Math.sin(radZ);
            double y1 = lx * Math.sin(radZ) + ly * Math.cos(radZ);
            double z1 = lz;

            // 绕 Y 轴
            double x2 = x1 * Math.cos(radY) + z1 * Math.sin(radY);
            double y2 = y1;
            double z2 = -x1 * Math.sin(radY) + z1 * Math.cos(radY);

            // 绕 X 轴
            double x3 = x2;
            double y3 = y2 * Math.cos(radX) - z2 * Math.sin(radX);
            double z3 = y2 * Math.sin(radX) + z2 * Math.cos(radX);

            // 3. 应用绝对 nodeMove 平移
            return new double[]{x3 + posX, y3 + posY, z3 + posZ};
        }
    }

    /**
     * Lightweight data structure to temporarily store valid parts during assembly
     */
    public static class PartEntry {
        JsonObject json;
        int partId;
        String partName;
        TransformContext transform;

        public Map<String, Double> variables;

        PartEntry(JsonObject j, int id, String name, TransformContext transform, Map<String, Double> variables) {
            this.json = j;
            this.partId = id;
            this.partName = name;
            this.transform = transform;
            this.variables = variables;
        }
    }

    public boolean assembleVehicle(
            String rootPartName,
            Map<String, String> userConfig,
            Map<String, JsonObject> registry,
            SoftBodyVehicle vehicle)
    {
        try {
            List<PartEntry> activeParts = new ArrayList<>();
            CouplerRegistry couplerRegistry = new CouplerRegistry();
            Map<String, String[]> vehicleRailMap = new HashMap<>();
            Map<String, Double> globalVariables = new HashMap<>();

            // Phase 0: Recursively collect all required parts before assembly
            JsonObject rootPart = registry.get(rootPartName);
            if (rootPart != null) {
                collectPartsRecursive(rootPartName, rootPart, userConfig, registry, activeParts, new TransformContext(), globalVariables);
            }

            System.out.println("====== 🛠️ Starting multi-Pass Assembly ======");
            System.out.println("Collected " + activeParts.size() + " valid part modules.");

            // Pass 1: Create all nodes FIRST
            for (PartEntry entry : activeParts) {
                if (entry.json.has("nodes")) {
                    JBeamParser.parseNodes(entry.json.getAsJsonArray("nodes"), vehicle, entry, couplerRegistry);
                }
            }
            // Internal camera rows define their own nodes and link to ordinary
            // nodes from any active part, so parse them after the node pass.
            for (PartEntry entry : activeParts) {
                if (entry.json.has("camerasInternal") && entry.json.get("camerasInternal").isJsonArray()) {
                    JBeamCameraParser.parseInternal(
                            entry.json.getAsJsonArray("camerasInternal"), vehicle, entry);
                }
                JBeamCameraParser.parseMetadata(entry.json, vehicle, entry);
            }
            System.out.println("✅ Pass 1 Complete: Nodes spawned | Total nodes: " + vehicle.nodes.count);

            // Pass 2: Build all structural connections (beams, surfaces, joints)
            for (PartEntry entry : activeParts) {
                if (entry.json.has("beams")) {
                    JBeamParser.parseBeams(entry.json.getAsJsonArray("beams"), vehicle, entry);
                }
                if (entry.json.has("hydros")) {
                    JBeamParser.parseHydros(entry.json.getAsJsonArray("hydros"), vehicle, entry);
                }
                if (entry.json.has("triangles")) {
                    JBeamParser.parseTriangles(entry.json.getAsJsonArray("triangles"), vehicle, entry);
                }
                if (entry.json.has("torsionbars")) {
                    JBeamParser.parseTorsionbars(entry.json.getAsJsonArray("torsionbars"), vehicle, entry);
                }
                if (entry.json.has("rails")) {
                    JBeamParser.parseRails(entry.json.getAsJsonObject("rails"), vehicleRailMap);
                }
                if (entry.json.has("slidenodes")) {
                    JBeamParser.parseSlidenodes(entry.json.getAsJsonArray("slidenodes"), vehicleRailMap, vehicle, entry);
                }
                if (entry.json.has("flexbodies")) {
                    // Flexbody 也需要接受空间变换矩阵以正确渲染位移
                    JBeamParser.parseFlexbodies(entry.json.getAsJsonArray("flexbodies"), vehicle, rootPartName, entry);
                }
            }
            int beamsCount = vehicle.normalBeams.count + vehicle.supportBeams.count + vehicle.boundedBeams.count;
            System.out.println("✅ Pass 2 Complete: Structures built | Total beams: " + beamsCount);

            // Pass 3: 逆向解析车轮
            System.out.println("====== 🛞 Assembling Wheels ======");
            JsonObject wheelConfigBlackboard = new JsonObject();
            for (PartEntry entry : activeParts) {
                if (entry.json.has("pressureWheels")) {
                    JBeamPressureWheelsParser.parsePressureWheels(
                            entry.json.getAsJsonArray("pressureWheels"),
                            vehicle,
                            entry,
                            wheelConfigBlackboard
                    );
                }
            }
            System.out.println("✅ Pass 3 Complete: Wheels generated.");

            // Powertrain specs are accumulated only after wheels exist, then
            // compiled once in finalizePhysicsSetup into hot-loop SoA arrays.
            for (PartEntry entry : activeParts) {
                vehicle.powertrain.addSpecs(JBeamPowertrainParser.parsePart(entry.json, entry.variables));
            }

            // Pass 4: Resolve Couplers
            System.out.println("====== 🔗 Resolving Couplers ======");
            int weldedCount = 0;
            for (CouplerRegistry.CouplerDef source : couplerRegistry.definitions) {
                if (source.couplerTag != null && !source.couplerTag.isEmpty()) {
                    CouplerRegistry.CouplerDef bestTarget = null;
                    double minDistanceSq = Double.MAX_VALUE;
                    double precompTime = 1.0;
                    double precompRange = 0.0;

                    Integer sourceIdx = vehicle.nodes.nameToIndex.get(source.nodeName);
                    if (sourceIdx == null) continue;
                    double sx = vehicle.nodes.posX[sourceIdx], sy = vehicle.nodes.posY[sourceIdx], sz = vehicle.nodes.posZ[sourceIdx];

                    for (CouplerRegistry.CouplerDef target : couplerRegistry.definitions) {
                        if (source != target && source.couplerTag.equals(target.tag)) {
                            Integer targetIdx = vehicle.nodes.nameToIndex.get(target.nodeName);
                            if (targetIdx == null) continue;

                            double dx = sx - vehicle.nodes.posX[targetIdx], dy = sy - vehicle.nodes.posY[targetIdx], dz = sz - vehicle.nodes.posZ[targetIdx];
                            double distSq = dx * dx + dy * dy + dz * dz;

                            if (distSq <= source.startRadius * source.startRadius && distSq < minDistanceSq) {
                                minDistanceSq = distSq;
                                bestTarget = target;
                                double dist = Math.sqrt(distSq);
                                double distanceToTravel = dist - source.lockRadius;

                                if (distanceToTravel > 0) {
                                    precompTime = distanceToTravel / Math.max(source.latchSpeed, 1e-12);
                                    precompRange = source.lockRadius;
                                }
                            }
                        }
                    }

                    if (bestTarget != null) {
                        double finalStrength = source.weld ? PhysicsWorld.KINDA_BIG_NUMBER : source.strength;
                        vehicle.addBeam(new PhysicsSpecs.BeamSpec(
                                BeamContainer.BEAM_NORMAL,
                                source.nodeName, bestTarget.nodeName, null,
                                null, 0, false,
                                1e9f, 1e7f,
                                PhysicsWorld.KINDA_BIG_NUMBER, (float) finalStrength,
                                0.0f, (float) precompRange, (float) precompTime,
                                0.0f, 0.0f, -1.0f, -1.0f,
                                0.0f, 0.0f,
                                -1.0f, -1.0f, -1.0f, -1.0f,
                                0.0f, 0.0f, 0.0f,
                                PhysicsWorld.KINDA_BIG_NUMBER
                        ));
                        weldedCount++;
                    }
                }
            }
            System.out.println("✅ Pass 4 Complete: " + weldedCount + " Couplers welded.");

            vehicle.finalizePhysicsSetup();

            return true;
        } catch (Throwable t) {
            System.err.println("🚨 车辆装配过程中发生严重错误！");
            t.printStackTrace();
            return false;
        }
    }

    private void collectPartsRecursive(String partName, JsonObject part, Map<String, String> userConfig, Map<String, JsonObject> registry, List<PartEntry> activeParts, TransformContext currentTransform, Map<String, Double> globalVariables) {
        currentPartId++;

        // 提取变量并存入上下文
        if (part.has("variables")) {
            JsonArray varsArray = part.getAsJsonArray("variables");
            boolean isHeader = true;
            for (JsonElement element : varsArray) {
                if (element.isJsonArray()) {
                    JsonArray row = element.getAsJsonArray();
                    if (isHeader) { isHeader = false; continue; } // 跳过表头
                    // JBeam 变量行标准格式: ["name", "type", "unit", "category", "default", ...]
                    if (row.size() >= 5) {
                        String varName = row.get(0).getAsString();
                        // 去掉前面的 "$" 符号，方便统一管理
                        if (varName.startsWith("$")) varName = varName.substring(1);
                        try {
                            float defVal = row.get(4).getAsFloat();
                            globalVariables.put(varName, (double) defVal);
                        } catch (Exception ignored) {}
                    }
                }
            }
        }

        activeParts.add(new PartEntry(part, currentPartId, partName, currentTransform, globalVariables));

        if (part.has("slots2")) {
            parseSlotsArray(part.getAsJsonArray("slots2"), partName, userConfig, registry, activeParts, currentTransform, globalVariables);
        }
        if (part.has("slots")) {
            parseSlotsArray(part.getAsJsonArray("slots"), partName, userConfig, registry, activeParts, currentTransform, globalVariables);
        }
    }

    private void parseSlotsArray(JsonArray slotsArray, String partName, Map<String, String> userConfig, Map<String, JsonObject> registry, List<PartEntry> activeParts, TransformContext parentTransform, Map<String, Double> globalVariables) {
        boolean isHeader = true;
        int typeIdx = 0;
        int defaultIdx = 1;

        for (JsonElement element : slotsArray) {
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();

                if (isHeader) {
                    for (int i = 0; i < row.size(); i++) {
                        String headerName = row.get(i).getAsString().toLowerCase();
                        if (headerName.equals("type") || headerName.equals("name")) typeIdx = i;
                        if (headerName.equals("default")) defaultIdx = i;
                    }
                    isHeader = false;
                    continue;
                }

                if (row.size() <= Math.max(typeIdx, defaultIdx)) continue;

                String slotName = row.get(typeIdx).getAsString();
                String defaultPart = row.get(defaultIdx).getAsString();
                String partToLoad = userConfig.getOrDefault(slotName, defaultPart);

                if (!partToLoad.equals("none") && !partToLoad.isEmpty()) {
                    JsonObject childPart = registry.get(partToLoad);
                    if (childPart != null) {
                        // 创建基于父节点的新变换上下文
                        TransformContext childTransform = new TransformContext(parentTransform);

                        if (row.get(row.size() - 1).isJsonObject()) {
                            JsonObject mod = row.get(row.size() - 1).getAsJsonObject();
                            Map<String, Double> vars = globalVariables;

                            // 1. 提取 nodeRotate (按照标准顺序首先生效旋转)
                            if (mod.has("nodeRotate")) {
                                JsonObject nr = mod.getAsJsonObject("nodeRotate");
                                Float rx = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(nr, "x", "0"), vars);
                                Float ry = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(nr, "y", "0"), vars);
                                Float rz = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(nr, "z", "0"), vars);
                                if (rx != null) childTransform.rotX += rx;
                                if (ry != null) childTransform.rotY += ry;
                                if (rz != null) childTransform.rotZ += rz;
                            }

                            // 2. 提取 nodeOffset (累加至对称镜像平移层)
                            if (mod.has("nodeOffset")) {
                                JsonObject no = mod.getAsJsonObject("nodeOffset");
                                Float ox = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(no, "x", "0"), vars);
                                Float oy = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(no, "y", "0"), vars);
                                Float oz = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(no, "z", "0"), vars);
                                if (ox != null) childTransform.offsetX += ox;
                                if (oy != null) childTransform.offsetY += oy;
                                if (oz != null) childTransform.offsetZ += oz;
                            }

                            // 3. 提取 nodeMove (累加至绝对方向平移层)
                            if (mod.has("nodeMove")) {
                                JsonObject nm = mod.getAsJsonObject("nodeMove");
                                Float mx = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(nm, "x", "0"), vars);
                                Float my = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(nm, "y", "0"), vars);
                                Float mz = JBeamParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(nm, "z", "0"), vars);
                                if (mx != null) childTransform.posX += mx;
                                if (my != null) childTransform.posY += my;
                                if (mz != null) childTransform.posZ += mz;
                            }
                        }
                        collectPartsRecursive(partToLoad, childPart, userConfig, registry, activeParts, childTransform, globalVariables);
                    } else {
                        System.err.println("🚨 Part [" + partName + "] slot [" + slotName + "] tried to load missing part: " + partToLoad);
                    }
                }
            }
        }
    }
}
