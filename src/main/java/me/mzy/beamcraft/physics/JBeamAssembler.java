package me.mzy.beamcraft.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Assembles a complete physical vehicle from JBeam part definitions
 * Uses a two-pass assembly process to ensure valid node connections before structural elements
 */
public class JBeamAssembler {
    private int currentPartId = 0;

    /**
     * Lightweight data structure to temporarily store valid parts during assembly
     * Holds the JSON definition, unique ID, and name of each part
     */
    private static class PartEntry {
        JsonObject json;
        int partId;
        String partName;
        double offX, offY, offZ;

        PartEntry(JsonObject j, int id, String name, double offX, double offY, double offZ) {
            this.json = j;
            this.partId = id;
            this.partName = name;
            this.offX = offX;
            this.offY = offY;
            this.offZ = offZ;
        }
    }

    /**
     * Main entry point for vehicle assembly
     * Collects all required parts and runs two-pass assembly to build the physics body
     */
    public void assembleVehicle(
            String rootPartName,
            Map<String, String> userConfig,
            Map<String, JsonObject> registry,
            SoftBodyVehicle vehicle)
    {
        List<PartEntry> activeParts = new ArrayList<>();
        CouplerRegistry couplerRegistry = new CouplerRegistry();

        // Phase 0: Recursively collect all required parts before assembly
        JsonObject rootPart = registry.get(rootPartName);
        if (rootPart != null) {
            collectPartsRecursive(rootPartName, rootPart, userConfig, registry, activeParts, 0.0, 0.0, 0.0);//根节点（车架）的初始偏移量是 0, 0, 0
        }

        System.out.println("====== 🛠️ Starting 3-Pass Assembly ======");
        System.out.println("Collected " + activeParts.size() + " valid part modules.");

        // Pass 1: Create all nodes FIRST
        // Critical: Nodes must exist before beams reference them to avoid broken connections
        for (PartEntry entry : activeParts) {
            if (entry.json.has("nodes")) {
                JBeamParser.parseNodes(entry.json.getAsJsonArray("nodes"), vehicle, entry.partId, couplerRegistry, entry.offX, entry.offY, entry.offZ);
            }
        }
        System.out.println("✅ Pass 1 Complete: Nodes spawned | Total nodes: " + vehicle.nodes.count);

        // Pass 2: Build all structural connections (beams, surfaces, joints)
        for (PartEntry entry : activeParts) {
            if (entry.json.has("beams")) {
                JBeamParser.parseBeams(entry.json.getAsJsonArray("beams"), vehicle, entry.partId);
            }

            // Treat hydraulic actuators as fixed beams for initial assembly
            if (entry.json.has("hydros")) {
                JBeamParser.parseBeams(entry.json.getAsJsonArray("hydros"), vehicle, entry.partId);
            }

            if (entry.json.has("triangles")) {
                JBeamParser.parseTriangles(entry.json.getAsJsonArray("triangles"), vehicle, entry.partId);
            }

            if (entry.json.has("torsionbars")) {
                JBeamParser.parseTorsionbars(entry.json.getAsJsonArray("torsionbars"), vehicle);
            }

            if (entry.json.has("rails")) {
                JBeamParser.parseRails(entry.json.getAsJsonObject("rails"));
            }

            if (entry.json.has("slidenodes")) {
                JBeamParser.parseSlidenodes(entry.json.getAsJsonArray("slidenodes"), vehicle);
            }

            if (entry.json.has("flexbodies")) {
                JBeamParser.parseFlexbodies(entry.json.getAsJsonArray("flexbodies"), vehicle, entry.partId);
            }
        }
        int beamsCount = vehicle.normalBeams.count + vehicle.supportBeams.count + vehicle.boundedBeams.count;
        System.out.println("✅ Pass 2 Complete: Structures built | Total beams: " + beamsCount);

        // ==========================================
        // 🚀 Pass 3: 逆向解析车轮 (解决子零件属性覆盖问题)
        // ==========================================
        System.out.println("====== 🛞 Assembling Wheels ======");
        // 1. 初始化黑板状态 (清除上一辆车的残留数据)
        JBeamPressureWheelsParser.resetBlackboard();

        for (PartEntry entry : activeParts) {
            if (entry.json.has("pressureWheels")) {
                // 2. 调用我们新写的解析器
                JBeamPressureWheelsParser.parsePressureWheels(entry.json.getAsJsonArray("pressureWheels"), vehicle);
            }
        }
        System.out.println("✅ Pass 3 Complete: Wheels generated.");

        // ==========================================
        // 🚀 Pass 4: Resolve Couplers (执行虚拟电焊)
        // ==========================================
        System.out.println("====== 🔗 Resolving Couplers ======");
        int weldedCount = 0;
        for (CouplerRegistry.CouplerDef source : couplerRegistry.definitions) {
            if (source.couplerTag != null && !source.couplerTag.isEmpty()) {
                CouplerRegistry.CouplerDef bestTarget = null;
                double minDistanceSq = Double.MAX_VALUE;
                double precompTime = 1.0;

                Integer sourceIdx = vehicle.nodes.nameToIndex.get(source.nodeName);
                if (sourceIdx == null) continue;
                double sx = vehicle.nodes.posX[sourceIdx], sy = vehicle.nodes.posY[sourceIdx], sz = vehicle.nodes.posZ[sourceIdx];

                for (CouplerRegistry.CouplerDef target : couplerRegistry.definitions) {
                    if (source != target && source.couplerTag.equals(target.tag)) {
                        Integer targetIdx = vehicle.nodes.nameToIndex.get(target.nodeName);
                        if (targetIdx == null) continue;

                        double dx = sx - vehicle.nodes.posX[targetIdx], dy = sy - vehicle.nodes.posY[targetIdx], dz = sz - vehicle.nodes.posZ[targetIdx];
                        double distSq = dx*dx + dy*dy + dz*dz;

                        // 半径检查 + 最近匹配
                        if (distSq <= source.startRadius * source.startRadius && distSq < minDistanceSq) {
                            minDistanceSq = distSq;
                            bestTarget = target;
                            double dist = Math.sqrt(distSq);
                            precompTime = dist / Math.max(source.latchSpeed, PhysicsWorld.KINDA_SMALL_NUMBER);
                        }
                    }
                }

                if (bestTarget != null) {
                    double finalStrength = source.weld ? PhysicsWorld.KINDA_BIG_NUMBER : source.strength;

                    // 🚀 生成 Coupler 虚拟梁
                    vehicle.addBeam(BeamContainer.BEAM_NORMAL,
                            source.nodeName, bestTarget.nodeName,
                            1e9, 1e7, // 超大刚度/阻尼，交由底层 maxSafe 自动截断
                            PhysicsWorld.KINDA_BIG_NUMBER, finalStrength,
                            0.0, 0.0, precompTime,  // precomp=0, precompTime=latchSpeed/dist
                            1.0, 1.0, -1.0, -1.0,
                            0.0, 0.0,
                            -1.0, -1.0, -1.0, -1.0
                    );
                    weldedCount++;
                }
            }
        }
        System.out.println("✅ Pass 4 Complete: " + weldedCount + " Couplers welded.");

        // Print final assembly manifest
        System.out.println("====== 📦 Active Parts Assembly List ======");
        for (PartEntry entry : activeParts) {
            System.out.println(" 🔧 " + entry.partName);
        }
    }

    /**
     * Recursively collects all dependent parts
     * Pure collection logic - no physics world modifications here
     */
    private void collectPartsRecursive(String partName, JsonObject part, Map<String, String> userConfig, Map<String, JsonObject> registry, List<PartEntry> activeParts, double offX, double offY, double offZ) {
        currentPartId++;
        activeParts.add(new PartEntry(part, currentPartId, partName, offX, offY, offZ));

        if (part.has("slots2")) {
            parseSlotsArray(part.getAsJsonArray("slots2"), partName, userConfig, registry, activeParts, offX, offY, offZ);
        }
        if (part.has("slots")) {
            parseSlotsArray(part.getAsJsonArray("slots"), partName, userConfig, registry, activeParts, offX, offY, offZ);
        }
    }

    /**
     * Parses slot arrays and loads child parts based on user config or defaults
     * Handles JBeam slot table format with headers and data rows
     */
    private void parseSlotsArray(JsonArray slotsArray, String partName, Map<String, String> userConfig, Map<String, JsonObject> registry, List<PartEntry> activeParts, double parentOffX, double parentOffY, double parentOffZ) {
        boolean isHeader = true;
        int typeIdx = 0;
        int defaultIdx = 1;

        for (JsonElement element : slotsArray) {
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();

                // First row = header: locate column indices for type and default
                if (isHeader) {
                    for (int i = 0; i < row.size(); i++) {
                        String headerName = row.get(i).getAsString().toLowerCase();
                        if (headerName.equals("type") || headerName.equals("name")) typeIdx = i;
                        if (headerName.equals("default")) defaultIdx = i;
                    }
                    isHeader = false;
                    continue;
                }

                // Skip invalid rows that don't have required columns
                if (row.size() <= Math.max(typeIdx, defaultIdx)) continue;

                String slotName = row.get(typeIdx).getAsString();
                String defaultPart = row.get(defaultIdx).getAsString();

                // Get user-selected part or fall back to default
                String partToLoad = userConfig.getOrDefault(slotName, defaultPart);

                // Load part if not disabled
                if (!partToLoad.equals("none") && !partToLoad.isEmpty()) {
                    JsonObject childPart = registry.get(partToLoad);
                    if (childPart != null) {
                        double slotOffX = 0, slotOffY = 0, slotOffZ = 0;
                        if (row.get(row.size() - 1).isJsonObject()) {
                            JsonObject mod = row.get(row.size() - 1).getAsJsonObject();
                            if (mod.has("nodeOffset")) {
                                JsonObject no = mod.getAsJsonObject("nodeOffset");
                                Double ox = JBeamPressureWheelsParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(no, "x", "0"), new java.util.HashMap<>());
                                Double oy = JBeamPressureWheelsParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(no, "y", "0"), new java.util.HashMap<>());
                                Double oz = JBeamPressureWheelsParser.evaluateBeamNGExpression(JBeamParser.getStringSafe(no, "z", "0"), new java.util.HashMap<>());
                                if (ox != null) slotOffX = ox;
                                if (oy != null) slotOffY = oy;
                                if (oz != null) slotOffZ = oz;
                            }
                        }
                        // 递归时，把父零件的绝对偏移量和当前插槽的局部偏移量累加
                        collectPartsRecursive(partToLoad, childPart, userConfig, registry, activeParts, parentOffX + slotOffX, parentOffY + slotOffY, parentOffZ + slotOffZ);
                    } else {
                        System.err.println("🚨 Part [" + partName + "] slot [" + slotName + "] tried to load missing part: " + partToLoad);
                    }
                }
            }
        }
    }
}