package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.assets.NamespaceScan;
import me.mzy.beamcraft.client.assets.ResolvedEntry;
import me.mzy.beamcraft.client.material.RelaxedJson;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;

public class JBeamLoader {

    public static String cleanJBeamSafe(String input) {
        return RelaxedJson.clean(input);
    }

    /**
     * @param vehiclesRootDir 模组车辆存放目录 (single-root convenience)
     * @param targetVehicleName 目标车辆内部名 (例如 "pickup")
     * @param pcFileName 配置文件名 (可选省略 .pc)
     */
    public static void loadVehicle(File vehiclesRootDir, String targetVehicleName, String pcFileName, Map<String, JsonObject> partRegistry, Map<String, String> userConfig) {
        loadVehicle(List.of(vehiclesRootDir), targetVehicleName, pcFileName, partRegistry, userConfig);
    }

    /**
     * Scans every configured asset root for the target vehicle's JBeam parts and
     * its {@code .pc} config, then resolves the config's {@code parts} into
     * {@code userConfig}. Discovery (including conflict resolution) is delegated
     * to {@link AssetScanner}; the vehicle name matches the inner
     * {@code vehicles/<name>/} path segment, not the outer container name.
     */
    public static void loadVehicle(List<File> assetRoots, String targetVehicleName, String pcFileName, Map<String, JsonObject> partRegistry, Map<String, String> userConfig) {
        // 处理可选的 .pc 后缀
        if (pcFileName != null && !pcFileName.isEmpty() && !pcFileName.endsWith(".pc")) {
            pcFileName += ".pc";
        }

        String[] pcContentBox = new String[]{""};
        int[] loadedCount = new int[]{0};

        System.out.println("====== 🔍 启动 JBeam 资产扫描 ======");

        // 1. 加载 common 资源 (common 里的 .pc 永不锁定)
        scanEntries(AssetScanner.INSTANCE.scan(assetRoots, "common"), null, partRegistry, pcContentBox, loadedCount);

        // 2. 扫描目标车辆
        scanEntries(AssetScanner.INSTANCE.scan(assetRoots, targetVehicleName), pcFileName, partRegistry, pcContentBox, loadedCount);

        System.out.println("📦 零件库加载完成。共读取 " + loadedCount[0] + " 个文件，提取 " + partRegistry.size() + " 个零件!");

        // 3. 解析 .pc 配置文件
        if (!pcContentBox[0].isEmpty()) {
            try {
                com.google.gson.stream.JsonReader pcReader = new com.google.gson.stream.JsonReader(new java.io.StringReader(pcContentBox[0]));
                pcReader.setLenient(true);
                JsonObject pcJson = JsonParser.parseReader(pcReader).getAsJsonObject();

                JsonObject parts = pcJson.has("parts") ? pcJson.getAsJsonObject("parts") : pcJson;
                for (String slot : parts.keySet()) {
                    userConfig.put(slot, parts.get(slot).getAsString());
                }
                System.out.println("📄 PC配置解析成功，载入 " + userConfig.size() + " 个插槽设定。");
            } catch (Exception e) {
                System.err.println("🚨 无法解析 .pc 配置文件结构");
                System.err.println(e.getMessage());
            }
        } else {
            System.err.println("⚠️ 未找到指定的 .pc 配置文件或其内容为空: " + pcFileName);
        }
    }

    private static void scanEntries(NamespaceScan scan, String targetPcName, Map<String, JsonObject> registry,
                                    String[] pcContentBox, int[] loadedCount) {
        for (ResolvedEntry entry : scan.entries()) {
            String logical = entry.logicalPath();
            if (!logical.endsWith(".jbeam") && !logical.endsWith(".pc")) {
                continue;
            }
            String fileName = basename(entry.entryName());
            try {
                String content = new String(entry.readBytes(), StandardCharsets.UTF_8);
                processFileContent(fileName, content, targetPcName, registry, pcContentBox, loadedCount);
            } catch (Exception e) {
                System.err.println(e.getMessage());
            }
        }
    }

    private static String basename(String path) {
        int slash = path.lastIndexOf('/');
        return slash >= 0 ? path.substring(slash + 1) : path;
    }

    private static void processFileContent(String fileName, String rawContent, String targetPcName, Map<String, JsonObject> registry, String[] pcContentBox, int[] loadedCount) {
        if (fileName.endsWith(".pc")) {
            if (targetPcName != null && fileName.equals(targetPcName)) {
                pcContentBox[0] = cleanJBeamSafe(rawContent);
                System.out.println("   🔒 Locked PC config: " + fileName);
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
