package me.mzy.beamcraft.client.material;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Immutable descriptor for one BeamNG material, derived from a
 * {@code *.materials.json} entry. Only the fields needed for a future
 * diffuse/alpha rendering pass are retained; PBR fields are intentionally not
 * captured yet.
 *
 * <p>Fields are keyed by the authoritative {@link #mapTo} in the index (see
 * {@link MaterialLibrary}), but the original name and {@code mapTo} casing are
 * preserved here for diagnostics.
 */
public final class MaterialDefinition {

    /** Material name from the {@code name} field, falling back to the JSON key. */
    public final String name;

    /** {@code mapTo} with original casing; falls back to the material name when absent. */
    public final String mapTo;

    /** {@code activeLayers}; defaults to 1 when absent. */
    public final int activeLayers;

    /** Up to four stages (unmodifiable). */
    public final List<MaterialStage> stages;

    /**
     * Material-level {@code baseColorFactor} or legacy {@code diffuseColor} as
     * an immutable RGBA value, or null when absent/unparseable. When the source
     * only had three components the alpha defaults to 1.
     *
     * <p>Note: in legacy BeamNG files {@code diffuseColor} is usually declared
     * per stage; see {@link MaterialStage#baseColorFactor}.
     */
    public final RgbaColor baseColorFactor;

    /** {@code alphaRef}; defaults to 0 when absent. */
    public final float alphaRef;

    /** {@code translucent}; defaults to false when absent. */
    public final boolean translucent;

    /** {@code translucentBlendOp} (e.g. "None", "Additive"); may be null. */
    public final String translucentBlendOp;

    /** {@code version} of the material JSON; 0 when absent. */
    public final float version;

    /** Archive/path diagnostic identifying where this material was defined. */
    public final String source;

    MaterialDefinition(String name, String mapTo, int activeLayers,
                       List<MaterialStage> stages, RgbaColor baseColorFactor,
                       float alphaRef, boolean translucent, String translucentBlendOp,
                       float version, String source) {
        this.name = name;
        this.mapTo = mapTo;
        this.activeLayers = activeLayers;
        this.stages = stages;
        this.baseColorFactor = baseColorFactor;
        this.alphaRef = alphaRef;
        this.translucent = translucent;
        this.translucentBlendOp = translucentBlendOp;
        this.version = version;
        this.source = source;
    }

    /**
     * Parses one material JSON object. Never throws; returns null only when the
     * input is unusable (callers then fail soft). Malformed fields degrade to
     * their defaults rather than rejecting the whole material.
     */
    public static MaterialDefinition fromJson(String materialName, JsonObject json, String source) {
        if (json == null) return null;

        String name = string(json, "name");
        if (name == null || name.isEmpty()) name = materialName;

        String mapTo = string(json, "mapTo");
        if (mapTo == null || mapTo.isEmpty()) mapTo = materialName;
        if (mapTo == null || mapTo.isEmpty()) return null;

        return new MaterialDefinition(
                name,
                mapTo,
                integer(json, "activeLayers", 1),
                parseStages(json.get("Stages")),
                parseColorFactor(json),
                number(json, "alphaRef", 0f),
                bool(json, "translucent", false),
                string(json, "translucentBlendOp"),
                number(json, "version", 0f),
                source
        );
    }

    private static List<MaterialStage> parseStages(JsonElement stagesElement) {
        if (stagesElement == null || !stagesElement.isJsonArray()) {
            return Collections.emptyList();
        }
        JsonArray array = stagesElement.getAsJsonArray();
        int limit = Math.min(array.size(), 4);
        List<MaterialStage> out = new ArrayList<>(limit);
        for (int i = 0; i < limit; i++) {
            JsonElement stage = array.get(i);
            if (stage == null || !stage.isJsonObject()) {
                continue;
            }
            JsonObject stageObject = stage.getAsJsonObject();
            String baseColorMap = string(stageObject, "baseColorMap");
            if (baseColorMap == null) {
                baseColorMap = string(stageObject, "colorMap"); // legacy name
            }
            String opacityMap = string(stageObject, "opacityMap");
            RgbaColor colorFactor = parseColorFactor(stageObject);
            out.add(new MaterialStage(i, baseColorMap, opacityMap, colorFactor));
        }
        return Collections.unmodifiableList(out);
    }

    private static RgbaColor parseColorFactor(JsonObject json) {
        JsonElement factor = json.get("baseColorFactor");
        if (factor == null) {
            factor = json.get("diffuseColor"); // legacy name
        }
        if (factor == null || factor.isJsonNull()) {
            return null;
        }
        try {
            float[] rgba = new float[4];
            int components = 0;
            if (factor.isJsonArray()) {
                JsonArray array = factor.getAsJsonArray();
                for (int i = 0; i < array.size() && i < 4; i++) {
                    JsonElement element = array.get(i);
                    if (element == null || element.isJsonNull()) {
                        break;
                    }
                    rgba[i] = element.getAsFloat();
                    components++;
                }
            } else if (factor.isJsonPrimitive() && factor.getAsJsonPrimitive().isString()) {
                String[] parts = factor.getAsString().trim().split("\\s+");
                for (int i = 0; i < parts.length && i < 4; i++) {
                    rgba[i] = Float.parseFloat(parts[i]);
                    components++;
                }
            } else {
                return null;
            }
            if (components == 0) {
                return null;
            }
            if (components == 3) {
                rgba[3] = 1f; // default alpha
            }
            return new RgbaColor(rgba[0], rgba[1], rgba[2], rgba[3]);
        } catch (Exception e) {
            return null;
        }
    }

    private static String string(JsonObject object, String key) {
        if (object == null || !object.has(key)) return null;
        JsonElement element = object.get(key);
        if (element == null || element.isJsonNull()) return null;
        try {
            return element.getAsString();
        } catch (Exception e) {
            return null;
        }
    }

    private static int integer(JsonObject object, String key, int defaultValue) {
        if (object == null || !object.has(key)) return defaultValue;
        JsonElement element = object.get(key);
        if (element == null || element.isJsonNull()) return defaultValue;
        try {
            return element.getAsInt();
        } catch (Exception e) {
            return defaultValue;
        }
    }

    private static float number(JsonObject object, String key, float defaultValue) {
        if (object == null || !object.has(key)) return defaultValue;
        JsonElement element = object.get(key);
        if (element == null || element.isJsonNull()) return defaultValue;
        try {
            return element.getAsFloat();
        } catch (Exception e) {
            return defaultValue;
        }
    }

    private static boolean bool(JsonObject object, String key, boolean defaultValue) {
        if (object == null || !object.has(key)) return defaultValue;
        JsonElement element = object.get(key);
        if (element == null || element.isJsonNull()) return defaultValue;
        try {
            return element.getAsBoolean();
        } catch (Exception e) {
            return defaultValue;
        }
    }
}
