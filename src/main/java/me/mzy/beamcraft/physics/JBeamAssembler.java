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

        PartEntry(JsonObject j, int id, String name) {
            this.json = j;
            this.partId = id;
            this.partName = name;
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

        // Phase 0: Recursively collect all required parts before assembly
        JsonObject rootPart = registry.get(rootPartName);
        if (rootPart != null) {
            collectPartsRecursive(rootPartName, rootPart, userConfig, registry, activeParts);
        }

        System.out.println("====== 🛠️ Starting Two-Pass Assembly ======");
        System.out.println("Collected " + activeParts.size() + " valid part modules.");

        // Pass 1: Create all nodes FIRST
        // Critical: Nodes must exist before beams reference them to avoid broken connections
        for (PartEntry entry : activeParts) {
            if (entry.json.has("nodes")) {
                JBeamParser.parseNodes(entry.json.getAsJsonArray("nodes"), vehicle, entry.partId);
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
        }
        System.out.println("✅ Pass 2 Complete: Structures built | Total beams: " + vehicle.beams.count);

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
    private void collectPartsRecursive(String partName, JsonObject part, Map<String, String> userConfig, Map<String, JsonObject> registry, List<PartEntry> activeParts) {
        currentPartId++;
        activeParts.add(new PartEntry(part, currentPartId, partName));

        if (part.has("slots3")) {
            parseSlotsArray(part.getAsJsonArray("slots3"), partName, userConfig, registry, activeParts);
        }
        if (part.has("slots2")) {
            parseSlotsArray(part.getAsJsonArray("slots2"), partName, userConfig, registry, activeParts);
        }
        if (part.has("slots1")) {
            parseSlotsArray(part.getAsJsonArray("slots1"), partName, userConfig, registry, activeParts);
        }
        if (part.has("slots")) {
            parseSlotsArray(part.getAsJsonArray("slots"), partName, userConfig, registry, activeParts);
        }
    }

    /**
     * Parses slot arrays and loads child parts based on user config or defaults
     * Handles JBeam slot table format with headers and data rows
     */
    private void parseSlotsArray(JsonArray slotsArray, String partName, Map<String, String> userConfig, Map<String, JsonObject> registry, List<PartEntry> activeParts) {
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
                        collectPartsRecursive(partToLoad, childPart, userConfig, registry, activeParts);
                    } else {
                        System.err.println("🚨 Part [" + partName + "] slot [" + slotName + "] tried to load missing part: " + partToLoad);
                    }
                }
            }
        }
    }
}