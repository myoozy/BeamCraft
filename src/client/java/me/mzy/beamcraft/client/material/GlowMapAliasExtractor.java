package me.mzy.beamcraft.client.material;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.Locale;
import java.util.Map;

/**
 * Backend-neutral extraction of <em>static</em> material aliases from BeamNG
 * JBeam {@code glowMap} sections.
 *
 * <p>A JBeam part carries a {@code glowMap} object whose keys are the
 * DAE/source material names of the part's meshes and whose values describe how
 * that material is lit at runtime, for example:
 * <pre>{@code
 *   "pickup_lowbeamglass": { "simpleFunction": "lowbeam_filament",
 *                            "off": "pickup_lightglass",
 *                            "on":  "pickup_lightglass_on" }
 * }</pre>
 * The {@code off} material is the name BeamNG resolves the mesh to while its
 * light is switched off, which is exactly what a static (lights-off) render
 * should draw. This extractor turns each {@code glowMap} key into an alias
 * {@code key -> off}, so a renderer that only knows the raw DAE material name
 * can look up the real material.
 *
 * <p><b>Scope</b>: static aliases only. Live emissive switching
 * ({@code on}/{@code on_intense}) and deformation switching
 * ({@code deformMaterialBase}/{@code deformMaterialDamaged}) are deliberately
 * not represented here; the latter is evidence that a fully generic parser would
 * need additional JBeam fields (see MaterialLibrary).
 *
 * <p><b>Rules</b> (deterministic, unit-tested): keys and targets are normalized
 * to lowercase (material lookups are case-insensitive everywhere in the
 * library); entries without a non-empty string {@code off} are skipped; values
 * that are not objects are skipped; a part that is not an object is skipped.
 * When the same key occurs twice in one scan, the last occurrence wins (the
 * caller controls ordering across files, so the collision rule stays
 * deterministic per archive/folder).
 *
 * <p>Parsing goes through {@link RelaxedJson}, the shared relaxed-JSON cleaner
 * (C-style comments, missing/trailing commas) that the JBeam and material loaders
 * already use; no fragile regex parsing is involved. A malformed file degrades
 * to no aliases and never throws.
 */
public final class GlowMapAliasExtractor {

    private GlowMapAliasExtractor() {
    }

    /**
     * Cleans and parses one relaxed-JSON JBeam file and appends its {@code
     * glowMap} {@code off} aliases into {@code out}. Never throws; a malformed
     * file contributes nothing.
     *
     * @param relaxedContent the raw file bytes as UTF-8 text (JBeam relaxed JSON)
     * @param out            target alias map, mutated in place (key and value
     *                       lowercase)
     */
    public static void collectFromJBeam(String relaxedContent, Map<String, String> out) {
        if (relaxedContent == null || out == null) {
            return;
        }
        JsonObject root;
        try {
            root = RelaxedJson.parse(relaxedContent);
        } catch (RuntimeException e) {
            System.err.println("⚠️ [Materials] Failed to parse JBeam content for material aliases: " + e.getMessage());
            return;
        }
        collect(root, out);
    }

    /**
     * Walks an already-parsed JBeam root object (top-level keys are parts) and
     * appends each part's {@code glowMap} {@code off} aliases into {@code out}.
     * Never throws.
     */
    public static void collect(JsonObject jbeamRoot, Map<String, String> out) {
        if (jbeamRoot == null || out == null) {
            return;
        }
        for (Map.Entry<String, JsonElement> partEntry : jbeamRoot.entrySet()) {
            JsonElement part = partEntry.getValue();
            if (part == null || !part.isJsonObject()) {
                continue;
            }
            JsonElement glowMap = part.getAsJsonObject().get("glowMap");
            if (glowMap == null || !glowMap.isJsonObject()) {
                continue;
            }
            collectFromGlowMap(glowMap.getAsJsonObject(), out);
        }
    }

    /**
     * Appends one {@code glowMap} object's {@code off} aliases into {@code out}.
     * Never throws. This is the smallest testable unit and mirrors exactly how
     * pickup/sunburst-style {@code glowMap} sections appear in real JBeam files.
     */
    public static void collectFromGlowMap(JsonObject glowMap, Map<String, String> out) {
        if (glowMap == null || out == null) {
            return;
        }
        for (Map.Entry<String, JsonElement> entry : glowMap.entrySet()) {
            JsonElement value = entry.getValue();
            if (value == null || !value.isJsonObject()) {
                continue;
            }
            String off = string(value.getAsJsonObject(), "off");
            if (off == null || off.isEmpty()) {
                continue;
            }
            String key = entry.getKey() == null ? null : entry.getKey().trim();
            if (key == null || key.isEmpty()) {
                continue;
            }
            out.put(key.toLowerCase(Locale.ROOT), off.toLowerCase(Locale.ROOT));
        }
    }

    private static String string(JsonObject object, String name) {
        if (object == null || !object.has(name)) {
            return null;
        }
        JsonElement element = object.get(name);
        // Only genuine JSON string values are material names; a numeric/boolean
        // primitive (e.g. a stray "off": 7) is malformed and must be skipped, not
        // coerced via getAsString().
        if (element == null || element.isJsonNull() || !element.isJsonPrimitive()
                || !element.getAsJsonPrimitive().isString()) {
            return null;
        }
        return element.getAsString();
    }
}
