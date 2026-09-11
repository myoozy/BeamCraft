package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Build-time parser for BeamNG {@code adaptiveDampers} controller declarations.
 *
 * <p>A part opts in through its {@code controller} table:
 *
 * <pre>
 * "controller": [
 *     ["fileName"],
 *     ["drivingDynamics/actuators/adaptiveDampers" {"name":"adaptiveRearDamper",
 *                                                  "dampBeamNames":["shock_RR", "shock_RL"]}]
 * ],
 * "adaptiveRearDamper": {
 *     "modes": [
 *         ["name",    "beamDampCoef", "beamDampFastCoef", "beamDampReboundCoef",
 *          "beamDampReboundFastCoef", "beamDampVelocitySplitCoef"],
 *         ["soft",    0.7,            1,                  0.65, 1, 0.7]
 *     ]
 * }
 * </pre>
 *
 * <p>Rows whose file name does not normalize to {@code drivingDynamics/actuators/adaptiveDampers}
 * are ignored, as are rows missing a resolvable instance name. Unknown sections,
 * malformed rows and a missing mode table all degrade to "no controller" rather
 * than failing the vehicle assembly.
 */
public final class AdaptiveDamperParser {
    /** Canonical, normalized controller file name this parser accepts. */
    public static final String CANONICAL_FILE_NAME = "drivingdynamics/actuators/adaptivedampers";

    private AdaptiveDamperParser() {}

    /**
     * Parses every adaptive damper controller declared by one (already unified)
     * JBeam part view.
     *
     * @param part      cleaned part JSON, typically the merged vehicle-wide view
     * @param variables part-scope variables for {@code $=...} evaluation
     */
    public static List<AdaptiveDamperSpec> parsePart(JsonObject part, Map<String, Double> variables) {
        if (part == null) return List.of();
        JsonElement controllerSection = part.get("controller");
        if (controllerSection == null || !controllerSection.isJsonArray()) return List.of();

        List<AdaptiveDamperSpec> specs = new ArrayList<>();
        for (JsonElement element : controllerSection.getAsJsonArray()) {
            if (!element.isJsonArray()) continue;
            AdaptiveDamperSpec spec = parseRow(part, element.getAsJsonArray(), variables);
            if (spec != null) specs.add(spec);
        }
        return specs;
    }

    private static AdaptiveDamperSpec parseRow(JsonObject part, JsonArray row,
                                               Map<String, Double> variables) {
        if (row.size() < 1) return null;
        if (!isAdaptiveDamperFileName(JBeamParser.getStringCell(row.get(0)))) return null;

        JsonObject options = null;
        for (int i = 1; i < row.size(); i++) {
            if (row.get(i).isJsonObject()) options = row.get(i).getAsJsonObject();
        }

        String instanceName = JBeamParser.getStringEvalSafe(options, "name", null, variables);
        if (instanceName == null || instanceName.isEmpty()) return null;

        JsonObject instance = part.has(instanceName) && part.get(instanceName).isJsonObject()
                ? part.getAsJsonObject(instanceName) : null;

        List<String> dampBeamNames = parseBeamNames(options, instance, variables);
        Map<String, AdaptiveDamperMode> modes = parseModes(instance, variables);
        if (modes.isEmpty()) return null;

        return new AdaptiveDamperSpec(instanceName, dampBeamNames, modes);
    }

    /**
     * {@code dampBeamNames} lives on the controller row in the stock ETK800 part;
     * the named controller object is accepted as a fallback for authors that put
     * it there instead (both end up in the controller's BeamNG data table).
     */
    private static List<String> parseBeamNames(JsonObject options, JsonObject instance,
                                               Map<String, Double> variables) {
        JsonArray rows = asArray(options == null ? null : options.get("dampBeamNames"));
        if (rows == null) rows = asArray(instance == null ? null : instance.get("dampBeamNames"));
        if (rows == null) return List.of();

        List<String> names = new ArrayList<>(rows.size());
        for (JsonElement element : rows) {
            String name = JBeamParser.getStringCell(element);
            if (name != null && !name.isEmpty() && !names.contains(name)) names.add(name);
        }
        return names;
    }

    /** Parses the {@code modes} header table; a missing or malformed table yields no modes. */
    private static Map<String, AdaptiveDamperMode> parseModes(JsonObject instance,
                                                              Map<String, Double> variables) {
        if (instance == null) return Map.of();
        JsonArray rows = asArray(instance.get("modes"));
        if (rows == null || rows.isEmpty()) return Map.of();

        Map<String, Integer> columns = columnsOf(rows.get(0));
        if (!columns.containsKey("name")) return Map.of();

        Map<String, AdaptiveDamperMode> modes = new LinkedHashMap<>();
        for (int r = 1; r < rows.size(); r++) {
            if (!rows.get(r).isJsonArray()) continue;
            AdaptiveDamperMode mode = parseModeRow(rows.get(r).getAsJsonArray(), columns, variables);
            if (mode != null) modes.put(mode.name(), mode);
        }
        return modes;
    }

    private static AdaptiveDamperMode parseModeRow(JsonArray row, Map<String, Integer> columns,
                                                   Map<String, Double> variables) {
        String name = modeName(row, columns);
        if (name == null || name.isEmpty()) return null;

        // A trailing inline object overrides the table cells, matching how every
        // other JBeam table in this codebase is read.
        JsonObject inline = null;
        int last = row.size() - 1;
        if (last > 0 && row.get(last).isJsonObject()) inline = row.get(last).getAsJsonObject();

        return new AdaptiveDamperMode(
                name,
                coef(row, columns, "beamDampCoef", inline, variables),
                coef(row, columns, "beamDampFastCoef", inline, variables),
                coef(row, columns, "beamDampReboundCoef", inline, variables),
                coef(row, columns, "beamDampReboundFastCoef", inline, variables),
                coef(row, columns, "beamDampVelocitySplitCoef", inline, variables));
    }

    private static float coef(JsonArray row, Map<String, Integer> columns, String key,
                              JsonObject inline, Map<String, Double> variables) {
        if (inline != null && inline.has(key)) {
            return JBeamParser.getFloatSafe(inline, key, AdaptiveDamperMode.DEFAULT_COEF, variables);
        }
        Integer index = columns.get(key.toLowerCase(Locale.ROOT));
        if (index == null || index < 0 || index >= row.size()) return AdaptiveDamperMode.DEFAULT_COEF;
        return (float) JBeamParser.getDoubleCell(
                row.get(index), AdaptiveDamperMode.DEFAULT_COEF, variables);
    }

    /**
     * The mode name must be an authored string. A number would still stringify,
     * which would silently register a mode nobody can address by name.
     */
    private static String modeName(JsonArray row, Map<String, Integer> columns) {
        Integer index = columns.get("name");
        if (index == null || index < 0 || index >= row.size()) return null;
        JsonElement cell = row.get(index);
        if (cell == null || !cell.isJsonPrimitive() || !cell.getAsJsonPrimitive().isString()) return null;
        return cell.getAsString();
    }

    private static Map<String, Integer> columnsOf(JsonElement headerRow) {
        Map<String, Integer> columns = new LinkedHashMap<>();
        if (headerRow == null || !headerRow.isJsonArray()) return columns;
        JsonArray row = headerRow.getAsJsonArray();
        for (int i = 0; i < row.size(); i++) {
            String cell = JBeamParser.getStringCell(row.get(i));
            if (cell != null && !cell.isEmpty()) columns.put(cell.toLowerCase(Locale.ROOT), i);
        }
        return columns;
    }

    private static JsonArray asArray(JsonElement element) {
        return element != null && element.isJsonArray() ? element.getAsJsonArray() : null;
    }

    /** True when {@code fileName} denotes the adaptive damper controller. */
    public static boolean isAdaptiveDamperFileName(String fileName) {
        String normalized = normalizeControllerFileName(fileName);
        if (normalized == null) return false;
        return normalized.equals(CANONICAL_FILE_NAME)
                || normalized.endsWith("/" + CANONICAL_FILE_NAME);
    }

    /**
     * Normalizes a JBeam controller file name: lower case, forward slashes, no
     * {@code .lua} extension, no leading or repeated separators. Returns
     * {@code null} for a blank input.
     */
    public static String normalizeControllerFileName(String fileName) {
        if (fileName == null) return null;
        String normalized = fileName.trim().replace('\\', '/').toLowerCase(Locale.ROOT);
        if (normalized.isEmpty()) return null;
        if (normalized.endsWith(".lua")) normalized = normalized.substring(0, normalized.length() - 4);
        while (normalized.startsWith("/")) normalized = normalized.substring(1);
        while (normalized.endsWith("/")) normalized = normalized.substring(0, normalized.length() - 1);
        while (normalized.contains("//")) normalized = normalized.replace("//", "/");
        return normalized.isEmpty() ? null : normalized;
    }
}
