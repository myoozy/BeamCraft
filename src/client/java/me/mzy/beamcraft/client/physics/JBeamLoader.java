package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.assets.NamespaceScan;
import me.mzy.beamcraft.client.assets.ResolvedEntry;
import me.mzy.beamcraft.client.debug.LoadTiming;
import me.mzy.beamcraft.client.material.RelaxedJson;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class JBeamLoader {

    public static String cleanJBeamSafe(String input) {
        return RelaxedJson.clean(input);
    }

    /**
     * @param vehiclesRootDir 模组车辆存放目录 (single-root convenience)
     * @param targetVehicleName 目标车辆内部名 (例如 "pickup")
     * @param pcFileName 配置文件名 (可选省略 .pc)
     * @return see {@link #loadVehicle(List, String, String, Map, Map)}
     */
    public static boolean loadVehicle(File vehiclesRootDir, String targetVehicleName, String pcFileName, Map<String, JsonObject> partRegistry, Map<String, String> userConfig) {
        return loadVehicle(List.of(vehiclesRootDir), targetVehicleName, pcFileName, partRegistry, userConfig);
    }

    /**
     * Scans every configured asset root for the target vehicle's JBeam parts and
     * its {@code .pc} config, then resolves the config's {@code parts} into
     * {@code userConfig}. Discovery (including conflict resolution) is delegated
     * to {@link AssetScanner}; the vehicle name matches the inner
     * {@code vehicles/<name>/} path segment, not the outer container name.
     *
     * <p>The config name is matched case-insensitively (an exact match still wins),
     * because vehicle folders are matched case-insensitively and most mods ship
     * {@code D15.pc} while users type {@code d15.pc}.
     *
     * @return {@code false} only when a {@code .pc} was named but could not be found
     *         or parsed. Callers must not assemble in that case: an empty
     *         {@code userConfig} does not fail — it quietly builds every slot from
     *         its default part, i.e. a vehicle other than the one that was asked for.
     *         Passing no config name at all is not a failure; that is the explicit
     *         "use every slot's default part" request.
     */
    public static boolean loadVehicle(List<File> assetRoots, String targetVehicleName, String pcFileName, Map<String, JsonObject> partRegistry, Map<String, String> userConfig) {
        String requestedPc = normalizePcName(pcFileName);

        // Keyed by the real (case-preserving) file name, because that is the name the
        // user has to type. Insertion order follows the scanner's logical-path
        // ordering, so a case-insensitive match stays deterministic. Every .pc is
        // kept here rather than only the requested one, so a miss can list the rest.
        Map<String, String> pcContents = new LinkedHashMap<>();
        int[] loadedCount = new int[]{0};

        System.out.println("Scanning JBeam assets");

        // 1. 加载 common 资源 (common 里的 .pc 永不锁定)
        long scanStart = LoadTiming.start();
        int commonFiles = scanEntries(AssetScanner.INSTANCE.scan(assetRoots, "common"), false, partRegistry, pcContents, loadedCount);
        LoadTiming.log("  [jbeam] common namespace, " + commonFiles + " files", scanStart);

        // 2. 扫描目标车辆
        scanStart = LoadTiming.start();
        int vehicleFiles = scanEntries(AssetScanner.INSTANCE.scan(assetRoots, targetVehicleName), true, partRegistry, pcContents, loadedCount);
        LoadTiming.log("  [jbeam] " + targetVehicleName + " namespace, " + vehicleFiles + " files", scanStart);

        System.out.println("Part library loaded: " + loadedCount[0] + " files, " + partRegistry.size() + " parts");

        // 3. 选定并解析 .pc 配置文件
        if (requestedPc == null) {
            return true;
        }

        String matchedPc = matchPcName(pcContents, requestedPc);
        if (matchedPc == null) {
            reportMissingPc(targetVehicleName, requestedPc, pcContents);
            return false;
        }

        try {
            com.google.gson.stream.JsonReader pcReader = new com.google.gson.stream.JsonReader(new java.io.StringReader(cleanJBeamSafe(pcContents.get(matchedPc))));
            pcReader.setLenient(true);
            JsonObject pcJson = JsonParser.parseReader(pcReader).getAsJsonObject();

            JsonObject parts = pcJson.has("parts") ? pcJson.getAsJsonObject("parts") : pcJson;
            for (String slot : parts.keySet()) {
                userConfig.put(slot, parts.get(slot).getAsString());
            }
            System.out.println("Parsed .pc config '" + matchedPc + "': " + userConfig.size() + " slot entries");
            return true;
        } catch (Exception e) {
            System.err.println("Failed to parse .pc config '" + matchedPc + "'");
            System.err.println(e.getMessage());
            return false;
        }
    }

    /**
     * Trims the requested name and appends {@code .pc} unless it already ends with
     * it. Case is preserved, so {@link #matchPcName} can still prefer an exact name
     * over a differently-cased variant.
     */
    private static String normalizePcName(String pcFileName) {
        if (pcFileName == null) {
            return null;
        }
        String trimmed = pcFileName.trim();
        if (trimmed.isEmpty()) {
            return null;
        }
        return trimmed.toLowerCase(Locale.ROOT).endsWith(".pc") ? trimmed : trimmed + ".pc";
    }

    /**
     * Resolves a requested name against the discovered {@code .pc} files: an exact
     * match wins, otherwise the first case-insensitive match in scan order.
     */
    private static String matchPcName(Map<String, String> pcContents, String requestedPc) {
        if (pcContents.containsKey(requestedPc)) {
            return requestedPc;
        }
        for (String candidate : pcContents.keySet()) {
            if (candidate.equalsIgnoreCase(requestedPc)) {
                return candidate;
            }
        }
        return null;
    }

    private static void reportMissingPc(String targetVehicleName, String requestedPc, Map<String, String> pcContents) {
        System.err.println("Requested .pc config '" + requestedPc + "' not found for vehicle '" + targetVehicleName + "'");
        if (pcContents.isEmpty()) {
            System.err.println("  this vehicle ships no .pc config; leave the config name off to use every slot's default part");
        } else {
            System.err.println("  available: " + String.join(", ", pcContents.keySet()));
        }
    }

    /**
     * Reads and parses every {@code .jbeam}/{@code .pc} entry in {@code scan}.
     *
     * @return the number of entries that were read and processed, for load timing
     */
    private static int scanEntries(NamespaceScan scan, boolean collectPc, Map<String, JsonObject> registry,
                                    Map<String, String> pcContents, int[] loadedCount) {
        int processed = 0;
        for (ResolvedEntry entry : scan.entries()) {
            // logicalPath() is the lowercased dedupe key, so this filter is already
            // case-insensitive; entryName() keeps the real casing.
            String logical = entry.logicalPath();
            if (!logical.endsWith(".jbeam") && !logical.endsWith(".pc")) {
                continue;
            }
            String fileName = basename(entry.entryName());
            long fileStart = LoadTiming.start();
            try {
                String content = new String(entry.readBytes(), StandardCharsets.UTF_8);
                processFileContent(fileName, content, collectPc, registry, pcContents, loadedCount);
            } catch (Exception e) {
                System.err.println(e.getMessage());
            }
            // Only files that stand out are reported: this loop runs over hundreds of
            // files, so a per-file line would bury the signal.
            LoadTiming.logSlowFile("  [jbeam] slow file " + logical, fileStart);
            processed++;
        }
        return processed;
    }

    private static String basename(String path) {
        int slash = path.lastIndexOf('/');
        return slash >= 0 ? path.substring(slash + 1) : path;
    }

    private static void processFileContent(String fileName, String rawContent, boolean collectPc,
                                           Map<String, JsonObject> registry, Map<String, String> pcContents,
                                           int[] loadedCount) {
        if (fileName.toLowerCase(Locale.ROOT).endsWith(".pc")) {
            // Collected, not parsed: selection happens after the whole scan so that a
            // miss can be reported against every name the vehicle actually ships.
            if (collectPc) {
                pcContents.putIfAbsent(fileName, rawContent);
            }
            return;
        }

        String cleanJson = cleanJBeamSafe(rawContent);
        try {
            com.google.gson.stream.JsonReader reader = new com.google.gson.stream.JsonReader(new java.io.StringReader(cleanJson));
            reader.setLenient(true);
            JsonObject fileJson = JsonParser.parseReader(reader).getAsJsonObject();

            for (String partName : fileJson.keySet()) {
                registry.put(partName, fileJson.getAsJsonObject(partName));
            }
            loadedCount[0]++;
        } catch (Exception e) {
            System.err.println(e.getMessage());
        }
    }
}
