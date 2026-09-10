package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Unifies the sections of selected JBeam parts into the vehicle-wide section view consumed by
 * configuration parsers. This is the section-unification phase, not variable/component
 * evaluation. Structural parsers still visit the original parts so they retain their per-slot
 * transform and origin metadata.
 */
public final class JBeamPartMerger {
    private static final Set<String> PART_TREE_ONLY_SECTIONS = Set.of("slots", "slots2", "information");

    private JBeamPartMerger() {
    }

    /** Parts must be supplied in assembly order: root first, then selected descendants. */
    public static JsonObject mergeParts(List<JsonObject> parts) {
        JsonObject vehicleData = new JsonObject();
        if (parts == null) return vehicleData;
        for (JsonObject part : parts) mergePartInto(vehicleData, part);
        return vehicleData;
    }

    public static void mergePartInto(JsonObject vehicleData, JsonObject part) {
        if (vehicleData == null || part == null) return;
        for (Map.Entry<String, JsonElement> entry : part.entrySet()) {
            String sectionName = entry.getKey();
            if (PART_TREE_ONLY_SECTIONS.contains(sectionName)) continue;

            JsonElement incoming = entry.getValue();
            JsonElement existing = vehicleData.get(sectionName);
            if (existing == null) {
                vehicleData.add(sectionName, incoming.deepCopy());
            } else if (existing.isJsonArray() && incoming.isJsonArray()) {
                appendTableRows(existing.getAsJsonArray(), incoming.getAsJsonArray());
            } else if (existing.isJsonObject() && incoming.isJsonObject()) {
                mergeSectionDictionary(existing.getAsJsonObject(), incoming.getAsJsonObject());
            }
            // BeamNG keeps the root value when two non-table sections collide.
        }
    }

    /** BeamNG table sections share a header, so only rows after the incoming header are appended. */
    private static void appendTableRows(JsonArray target, JsonArray source) {
        for (int i = Math.min(1, source.size()); i < source.size(); i++) {
            target.add(source.get(i).deepCopy());
        }
    }

    /**
     * Matches slotSystem.unifyParts dictionary semantics: ordinary keys overwrite, while numeric
     * merge modifiers operate on an existing numeric value and otherwise remain available for a
     * later parent/device-level merge.
     */
    private static void mergeSectionDictionary(JsonObject target, JsonObject source) {
        for (Map.Entry<String, JsonElement> entry : source.entrySet()) {
            String key = entry.getKey();
            JsonElement value = entry.getValue();
            if (isNumericMergeModifier(key, value)) {
                String actualKey = key.substring(2);
                JsonElement existing = target.get(actualKey);
                JsonElement existingModifier = target.get(key);
                if (isNumber(existingModifier)) {
                    existing = existingModifier;
                    actualKey = key;
                }
                if (isNumber(existing)) {
                    target.addProperty(actualKey,
                            applyNumericModifier(existing.getAsDouble(), key.charAt(1), value.getAsDouble()));
                } else {
                    target.add(key, value.deepCopy());
                }
            } else {
                target.add(key, value.deepCopy());
            }
        }
    }

    private static boolean isNumericMergeModifier(String key, JsonElement value) {
        if (key.length() < 3 || key.charAt(0) != '$' || !isNumber(value)) return false;
        return switch (key.charAt(1)) {
            case '+', '*', '<', '>' -> true;
            default -> false;
        };
    }

    private static boolean isNumber(JsonElement value) {
        return value != null && value.isJsonPrimitive() && value.getAsJsonPrimitive().isNumber();
    }

    private static double applyNumericModifier(double base, char operation, double value) {
        return switch (operation) {
            case '+' -> base + value;
            case '*' -> base * value;
            case '<' -> Math.min(base, value);
            case '>' -> Math.max(base, value);
            default -> base;
        };
    }
}
