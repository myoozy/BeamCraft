package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.material.RelaxedJson;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.stream.Stream;

public class JBeamLoader {

    public static String cleanJBeamSafe(String input) {
        return RelaxedJson.clean(input);
    }

    private static String readInputStream(InputStream is) throws Exception {
        java.io.ByteArrayOutputStream baos = new java.io.ByteArrayOutputStream();
        byte[] buffer = new byte[2048];
        int len;
        while ((len = is.read(buffer)) > 0) {
            baos.write(buffer, 0, len);
        }
        return baos.toString(java.nio.charset.StandardCharsets.UTF_8.name());
    }

    /**
     * @param vehiclesRootDir 模组车辆存放目录
     * @param targetVehicleName 目标车辆内部名 (例如 "pickup")
     * @param pcFileName 配置文件名 (可选省略 .pc)
     */
    public static void loadVehicle(File vehiclesRootDir, String targetVehicleName, String pcFileName, Map<String, JsonObject> partRegistry, Map<String, String> userConfig) {
        if (!vehiclesRootDir.exists()) {
            vehiclesRootDir.mkdirs();
            System.out.println("📁 Created vehicles directory at: " + vehiclesRootDir.getAbsolutePath());
            return;
        }

        // 处理可选的 .pc 后缀
        if (pcFileName != null && !pcFileName.isEmpty() && !pcFileName.endsWith(".pc")) {
            pcFileName += ".pc";
        }

        String[] pcContentBox = new String[]{""};
        int[] loadedCount = new int[]{0};

        System.out.println("====== 🔍 启动 JBeam 资产扫描 ======");

        // 1. 加载 common 资源
        File commonZip = new File(vehiclesRootDir, "common.zip");
        File commonDir = new File(vehiclesRootDir, "common");
        if (commonZip.exists()) scanZip(commonZip, targetVehicleName, null, partRegistry, pcContentBox, loadedCount, true);
        if (commonDir.exists()) scanFolder(commonDir, targetVehicleName, null, partRegistry, pcContentBox, loadedCount, true);

        // TODO: 未来可加入自动探测 Steam 安装路径及 AppData 用户配置路径 (当前先使用传入的 vehiclesRootDir)

        // 2. 扫描 vehiclesRootDir 下的其他文件寻找目标车辆的路径
        File[] files = vehiclesRootDir.listFiles();
        if (files != null) {
            for (File file : files) {
                String name = file.getName();
                if (name.equals("common.zip") || name.equals("common")) continue;

                // 只要压缩包/文件夹的名字里不包含我们要找的车（比如 "sunburst"），直接跳过
                if (name.toLowerCase().contains(targetVehicleName.toLowerCase())) {
                    if (file.isDirectory()) {
                        scanFolder(file, targetVehicleName, pcFileName, partRegistry, pcContentBox, loadedCount, false);
                    } else if (name.endsWith(".zip")) {
                        scanZip(file, targetVehicleName, pcFileName, partRegistry, pcContentBox, loadedCount, false);
                    }
                }
            }
        }

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

    private static void scanFolder(File folder, String targetVehicleName, String targetPcName, Map<String, JsonObject> registry, String[] pcContentBox, int[] loadedCount, boolean isCommon) {
        try (Stream<Path> paths = Files.walk(folder.toPath())) {
            paths.filter(Files::isRegularFile).forEach(path -> {
                String filePath = path.toString().replace("\\", "/");

                // 仅处理属于 common 或目标车辆路径的文件
                boolean isTarget = isCommon || filePath.contains("/vehicles/" + targetVehicleName + "/");

                if (isTarget && (filePath.endsWith(".jbeam") || filePath.endsWith(".pc"))) {
                    try (FileInputStream fis = new FileInputStream(path.toFile())) {
                        String content = readInputStream(fis);
                        String fileName = path.getFileName().toString();
                        processFileContent(fileName, content, targetPcName, registry, pcContentBox, loadedCount);
                    } catch (Exception e) {}
                }
            });
        } catch (Exception e) {
            System.err.println("🚨 Failed to walk directory: " + folder.getName());
            System.err.println(e.getMessage());
        }
    }

    private static void scanZip(File zipFile, String targetVehicleName, String targetPcName, Map<String, JsonObject> registry, String[] pcContentBox, int[] loadedCount, boolean isCommon) {
        try (java.util.zip.ZipFile zf = new java.util.zip.ZipFile(zipFile)) {
            java.util.Enumeration<? extends java.util.zip.ZipEntry> entries = zf.entries();

            while (entries.hasMoreElements()) {
                java.util.zip.ZipEntry entry = entries.nextElement();
                String name = entry.getName();

                // 只有文件名匹配，且不是目录/Mac垃圾文件时，才去真正解压读取数据
                boolean isTarget = isCommon || name.contains("vehicles/" + targetVehicleName + "/");

                if (isTarget && !entry.isDirectory() && !name.contains("__MACOSX") && (name.endsWith(".jbeam") || name.endsWith(".pc"))) {

                    try (InputStream is = zf.getInputStream(entry)) {
                        String content = readInputStream(is);

                        // 提取纯文件名用于后续匹配
                        String[] parts = name.split("/");
                        String fileName = parts[parts.length - 1];
                        processFileContent(fileName, content, targetPcName, registry, pcContentBox, loadedCount);
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("🚨 Failed to read ZIP: " + zipFile.getName());
            System.err.println(e.getMessage());
        }
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