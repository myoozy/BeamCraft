package me.mzy.beamcraft.physics;

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
        StringBuilder out = new StringBuilder();
        int len = input.length();
        int i = 0;

        // state machine
        boolean lastWasValueEnder = false; // Is the previous character the end of a value (e.g., the end of a number, a quotation mark, }, or ])?
        boolean lastWasComma = false;      // Was the last character entered a comma?

        while (i < len) {
            char c = input.charAt(i);

            // 1. skip all spaces
            if (Character.isWhitespace(c)) {
                i++; continue;
            }

            // 2. skip comments
            if (c == '/' && i + 1 < len) {
                char nc = input.charAt(i + 1);
                if (nc == '/') {
                    i += 2; // skip "//"
                    while (i < len && input.charAt(i) != '\n' && input.charAt(i) != '\r') i++;
                    continue;
                } else if (nc == '*') {
                    i += 2; // skip "/*"
                    while (i + 1 < len && !(input.charAt(i) == '*' && input.charAt(i + 1) == '/')) i++;
                    i += 2; // skip "*/"
                    continue;
                }
            }

            // 3. Automatically insert missing commas
            // If a quotation mark, minus sign, number, letter, {, or [ is encountered, it indicates that a new value is about to begin
            boolean currIsStarter = (c == '"' || c == '{' || c == '[' || c == '-' || c == '.' || Character.isLetterOrDigit(c));

            // If the previous value just ended and a new value starts here, there must be a missing comma in between!
            if (lastWasValueEnder && currIsStarter) {
                out.append(",\n");
                lastWasComma = true;
            }

            // 4. Process the string (without altering its contents)
            if (c == '"') {
                out.append(c);
                i++;
                while (i < len) {
                    char sc = input.charAt(i);
                    out.append(sc);
                    if (sc == '\\' && i + 1 < len) { // process \"
                        i++; out.append(input.charAt(i));
                    } else if (sc == '"') {
                        i++; break; // string end
                    }
                    i++;
                }
                lastWasValueEnder = true;
                lastWasComma = false;
                continue;
            }

            // 5. Processing structural symbols
            if (c == '{' || c == '[') {
                out.append(c);
                lastWasValueEnder = false; lastWasComma = false;
                i++; continue;
            }

            if (c == '}' || c == ']') {
                // Remove trailing commas (e.g., [1, 2, ] → [1, 2])
                if (lastWasComma) {
                    for (int j = out.length() - 1; j >= 0; j--) {
                        if (out.charAt(j) == ',') {
                            out.deleteCharAt(j);
                            break;
                        }
                    }
                }
                out.append(c);
                lastWasValueEnder = true; lastWasComma = false;
                i++; continue;
            }

            if (c == ':') {
                out.append(c);
                lastWasValueEnder = false; lastWasComma = false;
                i++; continue;
            }

            if (c == ',') {
                if (!lastWasComma && lastWasValueEnder) { // Prevent the occurrence of consecutive commas (, ,)
                    out.append(c);
                    lastWasComma = true;
                }
                lastWasValueEnder = false;
                i++; continue;
            }

            // 6. Handling unquoted words (numbers, true/false, or even unquoted keys)
            if (Character.isLetterOrDigit(c) || c == '-' || c == '.' || c == '+') {
                int start = i;
                while (i < len) {
                    char lc = input.charAt(i);
                    // Read through to the end, as long as the character is allowed in a word
                    if (Character.isLetterOrDigit(lc) || lc == '-' || lc == '.' || lc == '+' || lc == '_') {
                        i++;
                    } else {
                        break;
                    }
                }
                String word = input.substring(start, i);

                // Determine whether this string is a valid number or Boolean value
                boolean isNumberOrBool = word.equals("true") || word.equals("false") || word.equals("null");
                if (!isNumberOrBool) {
                    try {
                        Double.parseDouble(word); // Anything that can be parsed as a double is a number (including values like .5, which are fully supported)
                        isNumberOrBool = true;
                    } catch (NumberFormatException e) {
                        isNumberOrBool = false;
                    }
                }

                // If it's a standard number, just enter it as is;
                // if it's a letter without quotation marks (such as “Key”), force it into quotes!
                if (isNumberOrBool) {
                    out.append(word);
                } else {
                    out.append('"').append(word).append('"');
                }

                lastWasValueEnder = true; lastWasComma = false;
                continue;
            }

            // Skip any other unknown characters to prevent dirty data from causing interference
            i++;
        }

        String outString = out.toString();

        // Fix numbers like .5 that don't have leading zeros
        outString = outString.replaceAll("(?<=[\\s,\\[\\{:])\\.([0-9]+)", "0.$1");
        outString = outString.replaceAll("(?<=[\\s,\\[\\{:])-\\.([0-9]+)", "-0.$1");

        return outString;
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

        // 2. 扫描 vehiclesRootDir 下的其他文件寻找目标车辆的路径
        File[] files = vehiclesRootDir.listFiles();
        if (files != null) {
            for (File file : files) {
                String name = file.getName();
                if (name.equals("common.zip") || name.equals("common")) continue;

                if (file.isDirectory()) {
                    scanFolder(file, targetVehicleName, pcFileName, partRegistry, pcContentBox, loadedCount, false);
                } else if (name.endsWith(".zip")) {
                    scanZip(file, targetVehicleName, pcFileName, partRegistry, pcContentBox, loadedCount, false);
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
        }
    }

    private static void scanZip(File zipFile, String targetVehicleName, String targetPcName, Map<String, JsonObject> registry, String[] pcContentBox, int[] loadedCount, boolean isCommon) {
        try (ZipInputStream zis = new ZipInputStream(new FileInputStream(zipFile))) {
            ZipEntry entry;
            while ((entry = zis.getNextEntry()) != null) {
                String name = entry.getName();

                // 仅处理属于 common 或目标车辆路径的文件
                boolean isTarget = isCommon || name.contains("vehicles/" + targetVehicleName + "/");

                if (isTarget && !entry.isDirectory() && !name.contains("__MACOSX") && (name.endsWith(".jbeam") || name.endsWith(".pc"))) {
                    String content = readInputStream(zis);

                    // 提取纯文件名用于后续匹配
                    String[] parts = name.split("/");
                    String fileName = parts[parts.length - 1];
                    processFileContent(fileName, content, targetPcName, registry, pcContentBox, loadedCount);
                }
            }
        } catch (Exception e) {
            System.err.println("🚨 Failed to read ZIP: " + zipFile.getName());
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
            // 忽略格式损坏的文件
        }
    }
}