package me.mzy.beamcraft.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

/**
 * Parses JBeam formatted JSON data into physics world elements
 * Provides safe value extraction and handles all JBeam component types
 */
public class JBeamParser {
    public static final java.util.Map<String, String[]> RAIL_MAP = new java.util.HashMap<>();

    // ==========================================
    // Industrial-grade safe data extraction layer
    // Core utility for preventing crashes from malformed data
    // ==========================================

    /**
     * Safely extracts a double value from a JSON object
     * Handles nulls, empty strings, and BeamNG special float constants
     */
    public static double getDoubleSafe(JsonObject obj, String key, double defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;

        String str = el.getAsString().trim();
        if (str.isEmpty()) return defaultValue;

        // Translate BeamNG special float constants
        if (str.contains("FLT_MAX") || str.contains("MAX_FLT")) return PhysicsWorld.KINDA_BIG_NUMBER;
        if (str.contains("FLT_MIN") || str.contains("MIN_FLT")) return PhysicsWorld.KINDA_SMALL_NUMBER;

        try {
            return Double.parseDouble(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    /**
     * Safely extracts a trimmed string from a JSON object
     */
    public static String getStringSafe(JsonObject obj, String key, String defaultValue) {
        if (obj == null || !obj.has(key)) return defaultValue;
        JsonElement el = obj.get(key);
        if (el.isJsonNull()) return defaultValue;
        return el.getAsString().trim();
    }


    // --- 1. Node Parsing ---
    /**
     * Parses node definitions and creates physical nodes
     * Handles coordinate system conversion and inline property modifiers
     */
    public static void parseNodes(JsonArray nodes, PhysicsWorld world, int partId) {
        boolean isHeader = true;

        // Default values from Rig of Rods
        double currentWeight = 50.0;
        double currentFriction = 0.5;
        boolean currentCollision = true;

        for (JsonElement element : nodes) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentWeight = getDoubleSafe(modifier, "nodeWeight", currentWeight);
                currentFriction = getDoubleSafe(modifier, "frictionCoef", currentFriction);
                if (modifier.has("collision")) {
                    currentCollision = modifier.get("collision").getAsBoolean();
                }
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() < 4) continue;

                // Apply inline properties from the end of the row
                double inlineWeight = currentWeight;
                double inlineFriction = currentFriction;
                boolean inlineCollision = currentCollision;
                if (row.get(row.size() - 1).isJsonObject()) {
                    JsonObject inline = row.get(row.size() - 1).getAsJsonObject();
                    inlineWeight = getDoubleSafe(inline, "nodeWeight", inlineWeight);
                    inlineFriction = getDoubleSafe(inline, "frictionCoef", inlineFriction);
                    if (inline.has("collision")) {
                        inlineCollision = inline.get("collision").getAsBoolean();
                    }
                }

                String id = row.get(0).getAsString();
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;

                try {
                    // Coordinate system conversion: flip X, swap Y and Z
                    x = -row.get(1).getAsDouble();
                    y = +row.get(3).getAsDouble();
                    z = +row.get(2).getAsDouble();
                } catch (Exception e) {
                    System.err.println("⚠️ Failed to parse node coordinates, skipping: " + id);
                    continue;
                }

                world.addNode(id, x, y, z, inlineWeight, inlineFriction, partId, inlineCollision);
            }
        }
    }

    // --- 2. Beam Parsing ---
    /**
     * Parses beam definitions and creates physical connections
     * Supports multiple beam types and inline property overrides
     */
    public static void parseBeams(JsonArray beams, PhysicsWorld world, int partId) {
        boolean isHeader = true;

        int currentType = BeamContainer.BEAM_NORMAL;

        // Default values from Rig of Rods
        double currentPrecomp = 1.0;
        double currentSpring = 9000000.0, currentDamp = 12000.0;
        double currentDeform = 400000.0;
        double currentStrength = 1000000.0;

        double currentShortBound = 1.0, currentLongBound = 1.0;
        double currentShortBoundRange = -1.0, currentLongBoundRange = -1.0;
        double currentLimitSpring = currentSpring, currentLimitDamp = currentDamp;

        for (JsonElement element : beams) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();

                String bt = getStringSafe(modifier, "beamType", "");
                if (!bt.isEmpty()) {
                    if (bt.equals("|NORMAL") || bt.equals("|HYDRO") || bt.equals("|ANISOTROPIC")) {
                        currentType = BeamContainer.BEAM_NORMAL;
                    } else if (bt.equals("|SUPPORT")) {
                        currentType = BeamContainer.BEAM_SUPPORT;
                    } else if (bt.equals("|BOUNDED")) {
                        currentType = BeamContainer.BEAM_BOUNDED;
                    } else if (bt.equals("|LBEAM")) {
                        currentType = BeamContainer.BEAM_NORMAL;
                    }
                }

                currentPrecomp = getDoubleSafe(modifier, "beamPrecompression", currentPrecomp);
                currentSpring = getDoubleSafe(modifier, "beamSpring", currentSpring);
                currentDamp = getDoubleSafe(modifier, "beamDamp", currentDamp);
                currentDeform = getDoubleSafe(modifier, "beamDeform", currentDeform);
                currentStrength = getDoubleSafe(modifier, "beamStrength", currentStrength);

                currentShortBound = getDoubleSafe(modifier, "beamShortBound", currentShortBound);
                currentLongBound = getDoubleSafe(modifier, "beamLongBound", currentLongBound);
                currentLimitSpring = getDoubleSafe(modifier, "beamLimitSpring", currentLimitSpring);
                currentLimitDamp = getDoubleSafe(modifier, "beamLimitDamp", currentLimitDamp);

                if (modifier.has("beamShortBound")) currentShortBoundRange = -1.0;
                if (modifier.has("beamLongBound")) currentLongBoundRange = -1.0;

                currentShortBoundRange = getDoubleSafe(modifier, "shortBoundRange", currentShortBoundRange);
                currentLongBoundRange = getDoubleSafe(modifier, "longBoundRange", currentLongBoundRange);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {

                    // Copy current state to local inline variables
                    int inlineType = currentType;
                    double inlineSpring = currentSpring, inlineDamp = currentDamp;
                    double inlineDeform = currentDeform, inlineStrength = currentStrength;
                    double inlinePrecomp = currentPrecomp;
                    double inlineShortB = currentShortBound, inlineLongB = currentLongBound;
                    double inlineShortBRange = currentShortBoundRange, inlineLongBRange = currentLongBoundRange;
                    double inlineLimitS = currentLimitSpring, inlineLimitD = currentLimitDamp;

                    // Apply inline properties from the end of the row
                    if (row.size() >= 3 && row.get(row.size() - 1).isJsonObject()) {
                        JsonObject inline = row.get(row.size() - 1).getAsJsonObject();

                        inlineSpring = getDoubleSafe(inline, "beamSpring", inlineSpring);
                        inlineDamp = getDoubleSafe(inline, "beamDamp", inlineDamp);
                        inlineDeform = getDoubleSafe(inline, "beamDeform", inlineDeform);
                        inlineStrength = getDoubleSafe(inline, "beamStrength", inlineStrength);
                        inlinePrecomp = getDoubleSafe(inline, "beamPrecompression", inlinePrecomp);

                        inlineShortB = getDoubleSafe(inline, "beamShortBound", inlineShortB);
                        inlineLongB = getDoubleSafe(inline, "beamLongBound", inlineLongB);
                        inlineShortBRange = getDoubleSafe(inline, "shortBoundRange", inlineShortBRange);
                        inlineLongBRange = getDoubleSafe(inline, "longBoundRange", inlineLongBRange);
                        inlineLimitS = getDoubleSafe(inline, "beamLimitSpring", inlineLimitS);
                        inlineLimitD = getDoubleSafe(inline, "beamLimitDamp", inlineLimitD);

                        String bt = getStringSafe(inline, "beamType", "");
                        if (!bt.isEmpty()) {
                            if (bt.equals("|NORMAL") || bt.equals("|HYDRO") || bt.equals("|ANISOTROPIC")) inlineType = BeamContainer.BEAM_NORMAL;
                            else if (bt.equals("|SUPPORT")) inlineType = BeamContainer.BEAM_SUPPORT;
                            else if (bt.equals("|BOUNDED")) inlineType = BeamContainer.BEAM_BOUNDED;
                            else if (bt.equals("|LBEAM")) inlineType = BeamContainer.BEAM_NORMAL;
                        }
                    }

                    String id1 = row.get(0).getAsString();
                    String id2 = row.get(1).getAsString();

                    world.addBeam(id1, id2,
                            inlineSpring, inlineDamp, inlineDeform, inlineStrength,
                            inlineType, inlinePrecomp, inlineShortB, inlineLongB,
                            inlineShortBRange, inlineLongBRange,
                            inlineLimitS, inlineLimitD);
                }
            }
        }
    }

    // --- 3. Triangle Parsing ---
    /**
     * Parses triangle collision surfaces
     */
    public static void parseTriangles(JsonArray triangles, PhysicsWorld world, int partId) {
        boolean isHeader = true;
        for (JsonElement element : triangles) {
            if (element.isJsonObject()) continue;
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 3) {
                    world.addTriangle(row.get(0).getAsString(), row.get(1).getAsString(), row.get(2).getAsString(), partId);
                }
            }
        }
    }

    // --- 4. Torsionbar Parsing ---
    /**
     * Parses torsion bar joint definitions
     */
    public static void parseTorsionbars(JsonArray torsionbars, PhysicsWorld world) {
        boolean isHeader = true;
        double currentSpring = 0.0, currentDamp = 0.0;
        double currentDeform = PhysicsWorld.KINDA_BIG_NUMBER;
        double currentStrength = PhysicsWorld.KINDA_BIG_NUMBER;

        for (JsonElement element : torsionbars) {
            if (element.isJsonObject()) {
                JsonObject modifier = element.getAsJsonObject();
                currentSpring = getDoubleSafe(modifier, "spring", currentSpring);
                currentDamp = getDoubleSafe(modifier, "damp", currentDamp);
                currentDeform = getDoubleSafe(modifier, "deform", currentDeform);
                currentStrength = getDoubleSafe(modifier, "beamStrength", currentStrength);
                continue;
            }

            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 4) {
                    world.addTorsionBar(row.get(0).getAsString(), row.get(1).getAsString(), row.get(2).getAsString(), row.get(3).getAsString(),
                            currentSpring, currentDamp, currentDeform, currentStrength);
                }
            }
        }
    }

    // --- 5. Rail Parsing ---
    /**
     * Parses rail definitions and stores node sequences
     */
    public static void parseRails(JsonObject railsObj) {
        for (String railName : railsObj.keySet()) {
            JsonObject rail = railsObj.getAsJsonObject(railName);
            if (rail.has("links:")) {
                JsonArray links = rail.getAsJsonArray("links:");
                if (links.size() >= 2) {
                    // Store complete rail node sequence
                    String[] arr = new String[links.size()];
                    for (int i = 0; i < links.size(); i++) {
                        arr[i] = links.get(i).getAsString();
                    }
                    RAIL_MAP.put(railName, arr);
                }
            }
        }
    }

    // --- 6. Slidenode Parsing ---
    /**
     * Parses sliding nodes that move along predefined rails
     */
    public static void parseSlidenodes(JsonArray slidenodes, PhysicsWorld world) {
        boolean isHeader = true;
        for (JsonElement element : slidenodes) {
            if (element.isJsonObject()) continue;
            if (element.isJsonArray()) {
                JsonArray row = element.getAsJsonArray();
                if (isHeader) { isHeader = false; continue; }
                if (row.size() >= 2) {
                    String nodeId = row.get(0).getAsString();
                    String railName = row.get(1).getAsString();

                    double spring = 0;
                    double damp = 0;

                    if (row.size() > 5 && !row.get(5).isJsonNull()) {
                        String sStr = row.get(5).getAsString().trim();
                        if (!sStr.isEmpty() && !sStr.contains("FLT")) {
                            try {
                                spring = Double.parseDouble(sStr);
                            } catch (Exception e) {}
                        }
                    }

                    String[] links = RAIL_MAP.get(railName);
                    if (links != null && links.length >= 2) {
                        world.addSlidenode(nodeId, links, spring, damp);
                    }
                }
            }
        }
    }
}